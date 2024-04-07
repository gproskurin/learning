#ifndef _my_sx1276_h_included_
#define _my_sx1276_h_included_

#ifdef TARGET_NRF52DK
#include "lib_nrf5.h"
#else
#include "lib_stm32.h"
#endif

#include <array>
#include <concepts>
#include <type_traits>


namespace sx1276 {


struct hwconf_t {
#ifdef TARGET_NRF52DK
	using pin_t = nrf5_lib::gpio::pin_t;
	using spi_t = NRF_SPIM_Type;
#else
	using pin_t = stm32_lib::gpio::pin_t;
	using spi_t = SPI_TypeDef;
#endif
	spi_t* spi = nullptr;

#ifndef TARGET_NRF52DK
	uint8_t spi_af; // TODO per spi pin
#endif

	pin_t pin_spi_nss;
	pin_t pin_spi_sck;
	pin_t pin_spi_miso;
	pin_t pin_spi_mosi;

	pin_t pin_dio0;

	//stm32_lib::gpio::gpio_pin_t pin_radio_reset;
	//stm32_lib::gpio::gpio_pin_t pin_sx1276_reset;

	//stm32_lib::gpio::gpio_pin_t pin_radio_tcxo_vcc;
	//stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_rx;
	//stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_boost;
	//stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_rfo;
};


enum cf_t : uint8_t {
	// for RegModemConfig1
	bw_khz_7_8 = 0b0000,
	bw_khz_10_4 = 0b0001,
	bw_khz_15_6 = 0b0010,
	bw_khz_20_8 = 0b0011,
	bw_khz_31_25 = 0b0100,
	bw_khz_41_7 = 0b0101,
	bw_khz_62_5 = 0b0110,
	bw_khz_125 = 0b0111,
	bw_khz_250 = 0b1000,
	bw_khz_500 = 0b1001,
	bw_Msk = 0b11110000,
	bw_Pos = 4,

	// error coding rate
	ecr_4_5 = 0b001,
	ecr_4_6 = 0b010,
	ecr_4_7 = 0b011,
	ecr_4_8 = 0b100,
	ecr_Msk = 0b1110,
	ecr_Pos = 1,

	sf_Pos = 4,

	crc_Pos = 2,

	implicit_header_Pos = 0
};


enum regs_t : uint8_t {
	Fifo = 0x00,
	OpMode = 0x01,
	FifoAddrPtr = 0x0D,
	FifoTxBaseAddr = 0x0E,
	FifoRxCurrentAddr = 0x10,
	IrqFlagsMask = 0x11,
	IrqFlags = 0x12,
	FifoRxBytesNb = 0x13,
	PayloadLength = 0x22,
	Version = 0x42,
};


enum reg_val_t : uint8_t {
	OpMode_LongRangeMode = 0b10000000,
	OpMode_AccessSharedReg = 0b01000000,
	OpMode_LowFrequencyModeOn = 0b00001000,
	OpMode_Mode_msk = 0b111,
	OpMode_Mode_pos = 0,
	OpMode_Mode_SLEEP = 0b000,
	OpMode_Mode_STDBY = 0b001,
	OpMode_Mode_FSTX = 0b010,
	OpMode_Mode_TX = 0b011,
	OpMode_Mode_FSRX = 0b100,
	OpMode_Mode_RXCONTINUOUS = 0b101,
	OpMode_Mode_RXSINGLE = 0b110,
	OpMode_Mode_CAD = 0b111,

	IrqFlags_RxDone = 1 << 6,
	IrqFlags_PayloadCrcError = 1 << 5,
	IrqFlags_TxDone = 1 << 3,
	IrqFlags_CadDone = 1 << 2,
	IrqFlags_CadDetected = 1 << 0,
};


inline
uint8_t opmode(uint8_t v, uint8_t op_mode)
{
	return (v & ~sx1276::reg_val_t::OpMode_Mode_msk) | (op_mode << sx1276::reg_val_t::OpMode_Mode_pos);
}


namespace detail {
	template <class T> std::decay_t<T> decay_copy(T&&);
}

template <class LoraConfig>
concept IsLoraConfig = requires(LoraConfig lc)
{
	{ detail::decay_copy(LoraConfig::freq) } -> std::same_as<uint32_t>;
	{ detail::decay_copy(LoraConfig::sf) } -> std::same_as<uint8_t>;
	{ detail::decay_copy(LoraConfig::crc) } -> std::same_as<bool>;
	{ detail::decay_copy(LoraConfig::bw) } -> std::same_as<cf_t>;
	{ detail::decay_copy(LoraConfig::ecr) } -> std::same_as<cf_t>;
	{ detail::decay_copy(LoraConfig::preamble_length) } -> std::same_as<uint16_t>;
	{ detail::decay_copy(LoraConfig::implicit_header) } -> std::same_as<bool>;
};

struct LoraConfig_1 {
	static constexpr uint32_t freq = 866000000;
	static constexpr uint8_t sf = 7;
	static constexpr bool crc = true;
	static constexpr cf_t bw = cf_t::bw_khz_125;
	static constexpr cf_t ecr = cf_t::ecr_4_5;
	static constexpr uint16_t preamble_length = 8;
	static constexpr bool implicit_header = false;
};


inline
void sleep_ns(int ns)
{
	for (volatile int i=0; i<(ns/10)+2; ++i) {} // TODO
}


template <typename Pin>
void init_radio_pin(const Pin& pin)
{
#ifdef TARGET_NRF52DK
	pin.set(
		nrf5_lib::gpio::dir_t::output,
		nrf5_lib::gpio::pull_t::no_pull
	);
#else
	pin.set(
		stm32_lib::gpio::mode_t::output,
		stm32_lib::gpio::otype_t::push_pull,
		stm32_lib::gpio::pupd_t::no_pupd
	);
#endif
}


template <typename Pin>
void reset_radio_pin(const Pin& pin)
{
	// pull the pin down
#ifdef TARGET_NRF52DK
	pin.set(
		nrf5_lib::gpio::state_t::lo,
		nrf5_lib::gpio::pull_t::no_pull,
		nrf5_lib::gpio::dir_t::output
	);
#else
	stm32_lib::gpio::set_state(pin, 0);
	pin.set(
		stm32_lib::gpio::mode_t::output,
		stm32_lib::gpio::otype_t::push_pull,
		stm32_lib::gpio::pupd_t::no_pupd
	);
#endif

	vTaskDelay(configTICK_RATE_HZ/10); // > 100 us

	// make floating
#ifdef TARGET_NRF52DK
	pin.set(nrf5_lib::gpio::dir_t::input);
#else
	pin.set(stm32_lib::gpio::mode_t::input);
#endif
	vTaskDelay(configTICK_RATE_HZ/10); // > 5 ms
}


inline
void spi_sx_init(const hwconf_t& hwc, bool fast)
{
#if defined(TARGET_STM32L072)
	stm32_lib::spi::init_pin_nss(hwc.pin_spi_nss);
	stm32_lib::spi::init_pins(
		hwc.pin_spi_mosi, hwc.spi_af,
		hwc.pin_spi_miso, hwc.spi_af,
		hwc.pin_spi_sck, hwc.spi_af
	);

	// CPOL=0 CPHA=0, Motorola mode
	hwc.spi->CR1 = 0;
	const uint32_t cr1 =
		((fast ? 0b001 : 0b111) << SPI_CR1_BR_Pos)
		| SPI_CR1_MSTR_Msk
		| SPI_CR1_SSM_Msk
		;
	hwc.spi->CR1 = cr1;

	hwc.spi->CR2 = SPI_CR2_SSOE;

	hwc.spi->CR1 = cr1 | SPI_CR1_SPE;
#elif defined TARGET_NRF52DK
	hwc.pin_spi_nss.set(
		nrf5_lib::gpio::state_t::hi,
		nrf5_lib::gpio::dir_t::output
	);
	hwc.pin_spi_sck.set(
		nrf5_lib::gpio::state_t::lo, // CPOL
		//nrf5_lib::gpio::pull_t::pd,
		nrf5_lib::gpio::dir_t::output
	);
	hwc.pin_spi_mosi.set(
		nrf5_lib::gpio::state_t::lo,
		nrf5_lib::gpio::pull_t::pd,
		nrf5_lib::gpio::dir_t::output
	);
	hwc.pin_spi_miso.set(
		nrf5_lib::gpio::dir_t::input
	);

	hwc.spi->ENABLE = 0;
	hwc.spi->INTENCLR = 0xFFFFFFFF;
	hwc.spi->CONFIG = 0b000; // LSB first
	hwc.spi->PSEL.SCK = hwc.pin_spi_sck.reg;
	hwc.spi->PSEL.MOSI = hwc.pin_spi_mosi.reg;
	hwc.spi->PSEL.MISO = hwc.pin_spi_miso.reg;
	hwc.spi->FREQUENCY = 0x02000000; // 125kbps
	hwc.spi->ENABLE = 7;
#else
#error "Unsupported target"
#endif
}


typedef void (*spi_write_t)(const uint8_t *tx_data, size_t size, uint8_t* rx_data);
typedef void (*spi_nss_t)(bool);

template <spi_write_t SpiWrite, spi_nss_t Nss>
class spi_sx1276_t {
	void nss_0() { Nss(false); sleep_ns(30); }

	void nss_1() { sleep_ns(100); Nss(true); sleep_ns(30); }

	uint8_t reg_impl(uint8_t reg, uint8_t val)
	{
		const std::array<uint8_t, 2> tx_buf{reg, val};
		std::array<uint8_t, 2> rx_buf;
		nss_0();
		SpiWrite(tx_buf.data(), tx_buf.size(), rx_buf.data());
		nss_1();
		return rx_buf[1];
	}

public:
	uint8_t get_reg(uint8_t reg) { return reg_impl(reg, 0); }
	uint8_t set_reg(uint8_t reg, uint8_t val) { return reg_impl(reg | 0x80, val); }

	void spi_write(const uint8_t* data, size_t size)
	{
		nss_0();
		SpiWrite(data, size, nullptr);
		nss_1();
	}

	void fifo_write(const uint8_t* buf, size_t buf_size)
	{
		set_reg(regs_t::PayloadLength, buf_size);
		set_reg(regs_t::FifoAddrPtr, 0x80);
		set_reg(regs_t::FifoTxBaseAddr, 0x80);

		nss_0();

		constexpr uint8_t reg = 0x00 | 0x80; // write to reg 0x00
		SpiWrite(&reg, 1, nullptr);
		SpiWrite(buf, buf_size, nullptr);

		nss_1();
	}

	uint8_t fifo_read(uint8_t* rx_buf)
	{
		const auto rx_size = get_reg(regs_t::FifoRxBytesNb);
		const auto fifo_addr = get_reg(regs_t::FifoRxCurrentAddr);
		set_reg(regs_t::FifoAddrPtr, fifo_addr);

		nss_0();
		const uint8_t r = regs_t::Fifo; // read from fifo
		SpiWrite(&r, 1, nullptr);
		SpiWrite(nullptr, rx_size, rx_buf);
		nss_1();

		return rx_size;
	}
};


template <IsLoraConfig LoraConfig>
constexpr uint32_t calc_freq_reg()
{
	constexpr uint64_t cpu_freq = 32000000;
	const uint64_t fr = uint64_t(LoraConfig::freq) * uint64_t(1 << 19) / cpu_freq;
	static_assert(fr <= 0xFFFFFF);
	return uint32_t(fr);
}


template <IsLoraConfig LoraConfig, typename Spi>
void lora_configure(Spi&& spi)
{
	// RegOpMode
	spi.set_reg(0x01, sx1276::opmode(0b10000000, sx1276::reg_val_t::OpMode_Mode_SLEEP)); // Lora, sleep mode

	// carrier frequency
	// RegFrMsb, RegFrMid, RegFrLsb
	{
		constexpr uint32_t freq_reg = calc_freq_reg<LoraConfig>();
		static_assert(freq_reg == 0xD88000); // TODO
		constexpr std::array<uint8_t, 4> burst_buf{
			0x06 | 0x80,
			uint8_t((freq_reg >> 16) & 0xFF), // msb
			uint8_t((freq_reg >> 8) & 0xFF), // mid
			uint8_t(freq_reg & 0xFF) // lsb
		};
		spi.spi_write(burst_buf.data(), burst_buf.size());
	}

	// RegModemConfig1 & RegModemConfig2
	{
		constexpr std::array<uint8_t, 3> burst_buf{
			0x1D | 0x80,
			(LoraConfig::bw << sx1276::cf_t::bw_Pos)
				| (LoraConfig::ecr << sx1276::cf_t::ecr_Pos)
				| ((LoraConfig::implicit_header ? 1 : 0) << sx1276::cf_t::implicit_header_Pos),
			(LoraConfig::sf << sx1276::cf_t::sf_Pos)
				| ((LoraConfig::crc ? 1 : 0) << sx1276::cf_t::crc_Pos)
		};
		spi.spi_write(burst_buf.data(), burst_buf.size());
	}

	// RegPreamble
	{
		static_assert((LoraConfig::preamble_length >> 16) == 0);
		constexpr std::array<uint8_t, 3> burst_buf{
			0x20 | 0x80,
			LoraConfig::preamble_length >> 8, // msb
			LoraConfig::preamble_length & 0xFF // lsb
		};
		spi.spi_write(burst_buf.data(), burst_buf.size());
	}

	spi.set_reg(sx1276::regs_t::IrqFlags, 0xFF); // clear all flags
}



} // namespace

#endif

