#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include <algorithm>
#include <array>


#define UART_ID uart0
#define SPI_ID spi0

constexpr uint pin_led = 25;

std::array<uint8_t, 220 * 1024> g_buffer_big;
std::array<uint8_t, 256> g_buffer_small;
constexpr size_t g_flash_page_size = 256;

constexpr uint pin_uart_tx = 0;
constexpr uint pin_uart_rx = 1;

constexpr uint pin_spi_rx = 4;
constexpr uint pin_spi_cs = 5;
constexpr uint pin_spi_sck = 6;
constexpr uint pin_spi_tx = 7;


struct spi_cmd_t {
	static constexpr uint8_t page_program = 0x02;
	static constexpr uint8_t read_status_1 = 0x05;
	static constexpr uint8_t write_enable = 0x06;
	static constexpr uint8_t fast_read = 0x0B;
	//static constexpr uint8_t read_jedec = 0x9F;
	static constexpr uint8_t erase_all = 0xC7;

	static constexpr uint8_t enable_reset = 0x66;
	static constexpr uint8_t reset_device = 0x99;
};

struct proto_t {
	static constexpr uint8_t cmd_ping = 0x10;
	static constexpr uint8_t cmd_checksum_only = 0x20;
	static constexpr uint8_t cmd_read_all = 0x30;
	static constexpr uint8_t cmd_erase_all = 0x40;
	static constexpr uint8_t cmd_write = 0x50;
	static constexpr uint8_t cmd_checksum_flash = 0x60;

	static constexpr uint8_t resp_error = 0xFD;
	static constexpr uint8_t resp_unknown_command = 0xFE;

	static uint8_t resp(uint8_t cmd) { return cmd + 0x80; }
};


namespace my_uart {
	void init()
	{
		uart_init(UART_ID, 921600);
		gpio_pull_up(pin_uart_tx);
		gpio_pull_up(pin_uart_rx);
		gpio_set_function(pin_uart_tx, UART_FUNCSEL_NUM(UART_ID, pin_uart_tx));
		gpio_set_function(pin_uart_rx, UART_FUNCSEL_NUM(UART_ID, pin_uart_rx));
	}

	void tx_n(const uint8_t* src, size_t len)
	{
		uart_write_blocking(UART_ID, src, len);
	}

	void rx_n(uint8_t* dst, size_t len)
	{
		uart_read_blocking(UART_ID, dst, len);
	}

	uint8_t rx_1()
	{
		uint8_t c;
		rx_n(&c, 1);
		return c;
	}

	uint32_t rx_4()
	{
		std::array<uint8_t, 4> buf;
		rx_n(buf.data(), buf.size());
		return (uint32_t(buf[0]) << 24) | (uint32_t(buf[1]) << 16) | (uint32_t(buf[2]) << 8) | uint32_t(buf[3]);
	}

	void tx_1(uint8_t c)
	{
		return tx_n(&c, 1);
	}

	void tx_4(uint32_t n)
	{
		const std::array<uint8_t, 4> buf{
			uint8_t(n >> 24),
			uint8_t((n >> 16) & 0xff),
			uint8_t((n >> 8) & 0xff),
			uint8_t(n & 0xff)
		};
		tx_n(buf.data(), buf.size());
	}

}


namespace my_spi {
	void init()
	{
		spi_init(SPI_ID, 4*1000*1000);

		gpio_init(pin_spi_cs);
		gpio_put(pin_spi_cs, 1);
		gpio_set_dir(pin_spi_cs, GPIO_OUT);

		gpio_set_function(pin_spi_rx, GPIO_FUNC_SPI);
		gpio_set_function(pin_spi_sck, GPIO_FUNC_SPI);
		gpio_set_function(pin_spi_tx, GPIO_FUNC_SPI);

		// wakeup flash
		sleep_ms(10);
		gpio_put(pin_spi_cs, 0);
		sleep_ms(10);
		gpio_put(pin_spi_cs, 1);
		sleep_ms(10);
	}

	void nss_0()
	{
		sleep_ms(1);
		gpio_put(pin_spi_cs, 0);
		sleep_ms(1);
	}

	void nss_1()
	{
		sleep_ms(1);
		gpio_put(pin_spi_cs, 1);
		sleep_ms(1);
	}

	uint8_t spi_get_status_1()
	{
		std::array<uint8_t, 2> buf{spi_cmd_t::read_status_1, 0};
		my_spi::nss_0();
		spi_write_read_blocking(SPI_ID, buf.data(), buf.data(), buf.size());
		my_spi::nss_1();
		return buf[1];
	}

	void spi_wait_not_busy()
	{
		for(;;) {
			auto const st = spi_get_status_1();
			bool const busy = st & 0b00000001; // S0
			if (!busy) {
				return;
			}
			sleep_ms(100);
		}
	}

	void spi_write_enable()
	{
		my_spi::nss_0();
		spi_write_blocking(SPI_ID, &spi_cmd_t::write_enable, 1);
		my_spi::nss_1();
	}

	size_t spi_flash_size()
	{
		return 2*1024*1024;
		/*
		std::array<uint8_t, 4> buf{spi_cmd_t::read_jedec, 0, 0, 0};
		my_spi::nss_0();
		spi_write_read_blocking(SPI_ID, buf.data(), buf.data(), buf.size());
		my_spi::nss_1();

		return 1UL << buf[3];
		*/
	}

	void spi_reset()
	{
		my_spi::nss_0();
		spi_write_blocking(SPI_ID, &spi_cmd_t::enable_reset, 1);
		my_spi::nss_1();

		my_spi::nss_0();
		spi_write_blocking(SPI_ID, &spi_cmd_t::reset_device, 1);
		my_spi::nss_1();

		my_spi::nss_0();
		sleep_ms(10);
		my_spi::nss_1();
	}
}


static
void blink_n(int n)
{
	sleep_ms(500);
	for (int i=0; i<n; ++i) {
		gpio_put(pin_led, 1);
		sleep_ms(100);
		gpio_put(pin_led, 0);
		sleep_ms(300);
	}
	sleep_ms(500);
}


class crc32_t {
	uint32_t crc_ = 0xFFFFFFFF;

	void update(uint8_t data)
	{
		for(size_t j=0; j<8; j++) {
			bool const b = (data ^ crc_) & 1;
			crc_ >>= 1;
			if (b) {
				crc_ = crc_ ^ 0xEDB88320;
			}
			data >>= 1;
		}
	}

public:
	void update(const uint8_t* data, size_t len)
	{
		for (size_t i=0; i<len; ++i) {
			update(data[i]);
		}
	}

	uint32_t get() const
	{
		return ~crc_;
	}
};


namespace impl {
	uint32_t read_flash_and_maybe_send(size_t size_total, bool send_uart)
	{
		constexpr std::array<uint8_t, 5> tx_cmd{
			spi_cmd_t::fast_read,
			0, 0, 0, // 24bit addr
			0 // dummy byte
		};

		crc32_t crc;

		my_spi::nss_0();
		spi_write_blocking(SPI_ID, tx_cmd.data(), tx_cmd.size());
		size_t size_done = 0;
		while (size_done < size_total) {
			auto const chunk_size = std::min(size_total - size_done, g_buffer_small.size());
			spi_read_blocking(SPI_ID, 0, g_buffer_small.data(), chunk_size);
			crc.update(g_buffer_small.data(), chunk_size);
			if (send_uart) {
				my_uart::tx_n(g_buffer_small.data(), chunk_size);
			}
			size_done += chunk_size;
		}
		my_spi::nss_1();

		return crc.get();
	}
}


void process_ping()
{
	my_uart::tx_1(proto_t::resp(proto_t::cmd_ping));
}


void process_checksum_only()
{
	auto const size_total = my_uart::rx_4();

	crc32_t crc;
	size_t size_done = 0;
	while (size_done < size_total) {
		auto const chunk_size = std::min<size_t>(g_buffer_small.size(), size_total - size_done);
		my_uart::rx_n(g_buffer_small.data(), chunk_size);
		crc.update(g_buffer_small.data(), chunk_size);
		size_done += chunk_size;
	}
	auto const crc_calculated = crc.get();

	my_uart::tx_1(proto_t::resp(proto_t::cmd_checksum_only));
	my_uart::tx_4(crc_calculated);
}


void process_read_all()
{
	auto const size_total = my_spi::spi_flash_size();

	my_uart::tx_1(proto_t::resp(proto_t::cmd_read_all));
	my_uart::tx_4(size_total);
	sleep_ms(100);

	auto const crc = impl::read_flash_and_maybe_send(size_total, true);
	my_uart::tx_4(crc);
}

void process_erase_all()
{
	my_spi::spi_write_enable();

	my_spi::nss_0();
	spi_write_blocking(SPI_ID, &spi_cmd_t::erase_all, 1);
	my_spi::nss_1();

	my_spi::spi_wait_not_busy();

	my_uart::tx_1(proto_t::resp(proto_t::cmd_erase_all));
}

void process_write()
{
	// read all data via uart, checksum, write
	static_assert(g_buffer_big.size() >= g_flash_page_size);

	auto const size_total = my_uart::rx_4();

	if (size_total > g_buffer_big.size()) {
		my_uart::tx_1(proto_t::resp_error);
		return;
	}

	my_uart::rx_n(g_buffer_big.data(), size_total);

	auto const crc_remote = my_uart::rx_4();
	{
		crc32_t crc;
		crc.update(g_buffer_big.data(), size_total);
		auto const crc_local = crc.get();
		if (crc_local != crc_remote) {
			my_uart::tx_1(proto_t::resp_error);
			return;
		}
	}
	my_uart::tx_1(proto_t::resp(proto_t::cmd_write));

	size_t size_done = 0;
	while (size_done < size_total) {
		auto const chunk_size = std::min<size_t>(g_flash_page_size, size_total - size_done);
		const std::array<uint8_t, 4> tx_cmd{
			spi_cmd_t::page_program,
			// 24bit addr
			uint8_t(size_done >> 16),
			uint8_t((size_done >> 8) & 0xff),
			uint8_t(size_done & 0xff)
		};

		my_spi::spi_write_enable();
		my_spi::spi_wait_not_busy();

		my_spi::nss_0();

		spi_write_blocking(SPI_ID, tx_cmd.data(), tx_cmd.size());
		spi_write_blocking(SPI_ID, g_buffer_big.data()+size_done, chunk_size);

		my_spi::nss_1();

		size_done += chunk_size;
	}
	my_uart::tx_1(proto_t::resp(proto_t::cmd_write));
}

void process_checksum_flash()
{
	auto const size_total = my_uart::rx_4();
	my_uart::tx_1(proto_t::resp(proto_t::cmd_checksum_flash));
	auto const crc = impl::read_flash_and_maybe_send(size_total, false);
	my_uart::tx_4(crc);
}


int main()
{
	// init LED
	gpio_init(pin_led);
	gpio_set_dir(pin_led, GPIO_OUT);
	gpio_put(pin_led, 1);
	sleep_ms(500);
	gpio_put(pin_led, 0);
	sleep_ms(1000);

	my_uart::init();
	my_spi::init();
	
	for(;;) {
		gpio_put(pin_led, 1);
		auto const cmd = my_uart::rx_1();
		gpio_put(pin_led, 0);
		sleep_ms(100);
		switch (cmd) {
			case proto_t::cmd_ping:
				process_ping();
				break;
			case proto_t::cmd_checksum_only: {
				process_checksum_only();
				break;
			}
			case proto_t::cmd_read_all: {
				process_read_all();
				break;
			}
			case proto_t::cmd_erase_all:
				process_erase_all();
				break;
			case proto_t::cmd_write: {
				process_write();
				break;
			}
			case proto_t::cmd_checksum_flash: {
				process_checksum_flash();
				break;
			}
			default:
				my_uart::tx_1(proto_t::resp_unknown_command);
				break;
		}
	}
	return 0;
}

