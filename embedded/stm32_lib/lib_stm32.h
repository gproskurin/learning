#ifndef _gpr_lib_stm32_included_
#define _gpr_lib_stm32_included_

#include "cmsis_device.h"

#include <stddef.h>
#include <stdint.h>
#include <algorithm>
#include <utility>


namespace {
	constexpr uint32_t mask1(int n) { return 1 << (n); }
	constexpr uint32_t mask2(int n) { return 0b11 << ((n) * 2); }
	constexpr uint32_t mask4(int n) { return 0b1111 << ((n) * 4); }
}


namespace stm32_lib {

namespace gpio {

#ifdef TARGET_STM32F103
enum class mode_t {
	input = 0b00,
	output_10mhz = 0b01,
	output_2mhz = 0b10,
	output_50mhz = 0b11
};

enum class cnf_t {
	input_analog = 0b00,
	input_float = 0b01,
	input_pupd = 0b10,
	input_reserved1 = 0b11,
	output_pushpull = 0b00,
	output_opendrain = 0b01,
	output_af_pushpull = 0b10,
	output_af_opendrain = 0b11
};
#else
enum class mode_t {
	input = 0b00,
	output = 0b01,
	af = 0b10,
	analog = 0b11
};

enum class otype_t {
	push_pull = 0,
	open_drain = 1
};

enum class speed_t {
	bits_00 = 0b00,
	bits_01 = 0b01,
	bits_10 = 0b10,
	bits_11 = 0b11
};

enum class pupd_t {
	no_pupd = 0b00,
	pu = 0b01,
	pd = 0b10
};

struct af_t {
	const uint8_t af_num;
	explicit constexpr af_t(uint8_t af) : af_num(af) {}
};
#endif

template <bool Invert>
struct pin_impl_t {
	uint32_t const gpio_base; // store addr to avoid "unsafe" type cast and keep it constexpr
	uint8_t const reg;
	constexpr pin_impl_t(uint32_t base_addr, uint8_t r) : gpio_base(base_addr), reg(r) {}

	constexpr GPIO_TypeDef* gpio() const { return reinterpret_cast<GPIO_TypeDef*>(gpio_base); }

	void set_state(bool s) const
	{
		uint32_t const mask = ((s ^ Invert) ? (1U << reg) : (1U << reg) << 16);
		gpio()->BSRR = mask;
	}

#ifdef TARGET_STM32F103
	void set(mode_t m, cnf_t c) const
	{
		const uint32_t reg_lo = ((reg < 8) ? reg : reg - 8);
		auto const cr = ((reg < 8) ? &gpio()->CRL : &gpio()->CRH);
		*cr =
			(*cr & ~mask4(reg_lo))
			| (
				((static_cast<uint32_t>(c) << 2) | static_cast<uint32_t>(m))
				<<
				(reg_lo * 4)
			);
	}
#else
	template <typename... Targs>
	void set(mode_t m, Targs... args) const
	{
		set_mode(m);
		set(args...);
	}

	template <typename... Targs>
	void set(otype_t ot, Targs... args) const
	{
		gpio()->OTYPER = (gpio()->OTYPER & ~mask1(reg)) | (static_cast<uint32_t>(ot) << reg);
		set(args...);
	}

	template <typename... Targs>
	void set(speed_t s, Targs... args) const
	{
		gpio()->OSPEEDR = (gpio()->OSPEEDR & ~mask2(reg)) | (static_cast<uint32_t>(s) << (reg * 2));
		set(args...);
	}

	template <typename... Targs>
	void set(pupd_t p, Targs... args) const
	{
		gpio()->PUPDR = (gpio()->PUPDR & ~mask2(reg)) | (static_cast<uint32_t>(p) << (reg * 2));
		set(args...);
	}

	template <typename... Targs>
	void set(af_t af, Targs... args) const
	{
		const auto reg_lo = (reg < 8) ? reg : (reg-8);
		auto const afr = &gpio()->AFR[ (reg<8) ? 0 : 1 ];
		*afr = (*afr & ~mask4(reg_lo)) | (af.af_num << (reg_lo * 4));
		set_mode(mode_t::af);
		set(args...);
	}

	void set_mode_output_lowspeed_pushpull() const
	{
		#ifdef TARGET_STM32F103
		set(mode_t::output_2mhz, cnf_t::output_pushpull);
		#else
		set(mode_t::output, otype_t::push_pull, speed_t::bits_00);
		#endif
	}

	void set_mode_button() const
	{
		set(mode_t::input, pupd_t::pu);
	}

	bool get_state() const
	{
		return gpio()->IDR & (1UL << reg);
	}

private:
	void set() const {} // terminate arguments recursion

	void set_mode(mode_t m) const
	{
		gpio()->MODER = (gpio()->MODER & ~mask2(reg)) | (static_cast<uint32_t>(m) << (reg * 2));
	}
#endif
};
using pin_t = pin_impl_t<false>;
using pin_inverted_t = pin_impl_t<true>;
using gpio_pin_t = pin_t; // compat


// compat
template <typename Pin>
void set_state(const Pin& pin, bool s)
{
	return pin.set_state(s);
}


inline
void set_mode_output_hispeed_pushpull(const gpio_pin_t& pin)
{
#ifdef TARGET_STM32F103
	#error
	//pin.set(mode_t::output_2mhz, cnf_t::output_pushpull);
#else
	pin.set(mode_t::output, otype_t::push_pull, speed_t::bits_11);
#endif
}


inline
#ifdef TARGET_STM32F103
void set_mode_af_lowspeed_pu(const gpio_pin_t& pin)
{
	// TODO pullup
	pin.set(mode_t::output_2mhz, cnf_t::output_af_pushpull);
}
#else
void set_mode_af_lowspeed_pu(const gpio_pin_t& pin, uint32_t af_num)
{
	pin.set(af_t(af_num), pupd_t::pu);
}
#endif


inline
#ifdef TARGET_STM32F103
void set_mode_af_lowspeed_pushpull(const gpio_pin_t& pin)
{
	pin.set(mode_t::output_2mhz, cnf_t::output_af_pushpull);
}
#else
void set_mode_af_lowspeed_pushpull(const gpio_pin_t& pin, uint32_t af_num)
{
	pin.set(af_t(af_num), otype_t::push_pull, speed_t::bits_00);
}
#endif

inline
void set_mode_output_analog(const gpio_pin_t& pin)
{
#ifdef TARGET_STM32F103
#error "TODO"
#else
	pin.set(mode_t::analog);
#endif
}


} // namespace gpio


namespace wwdg {


template <uint32_t CpuClockHz, uint32_t TickRateHz, uint32_t Psc, uint32_t Counts>
constexpr
uint32_t counts_to_ticks()
{
	static_assert(Psc==1 || Psc==2 || Psc==4 || Psc==8);
	static_assert(Counts <= (0x7F - 0x40));
	constexpr uint32_t wwdg_clock = CpuClockHz / 4096 / Psc;
	//static_assert(wwdg_clock * Psc * 4096 == CpuClockHz); // ensure no truncation
	constexpr uint32_t r1 = Counts * TickRateHz / wwdg_clock;
	//static_assert(r * wwdg_clock / TickRateHz == Counts);
	constexpr uint32_t r = Counts * TickRateHz * 4096 * Psc / CpuClockHz;
	static_assert(r1 == r);
	return r;
}


} // namespace wwdg


namespace usart {


namespace impl {
	constexpr uint32_t baudrate = 115200;
#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
	constexpr uint32_t cr1 = USART_CR1_TE | USART_CR1_FIFOEN;
#else
	constexpr uint32_t cr1 = USART_CR1_TE;
#endif
	constexpr uint32_t cr1_en = cr1 | USART_CR1_UE;

	template <uint32_t CpuClockHz>
	constexpr std::pair<uint32_t, uint32_t> lpuart_presc_brr()
	{
		constexpr uint64_t psc = 4;
		constexpr uint32_t psc_reg_val = 0b0010;
		constexpr uint64_t clk = CpuClockHz / psc;
		constexpr uint64_t brr = clk*256 / uint64_t(baudrate);
		static_assert(brr >= 0x300);
		static_assert(brr < (1 << 20));
		static_assert(3*baudrate < clk);
		static_assert(clk < 4096*baudrate);
		constexpr uint32_t brr_reg_val = brr;
		return std::make_pair(psc_reg_val, brr);
	}
}

template <uint32_t CpuClockHz>
void init_logger_uart(
	USART_TypeDef* const usart,
	const stm32_lib::gpio::pin_t& pin_tx,
	uint8_t af
)
{
	usart->CR1 = 0; // ensure UE flag is reset
	stm32_lib::gpio::set_mode_af_lowspeed_pu(pin_tx, af);
	constexpr auto brr = CpuClockHz / impl::baudrate;
	usart->BRR = brr;
	usart->CR1 = impl::cr1;
	usart->CR1 = impl::cr1_en;
}

#ifdef TARGET_STM32WL55
template <uint32_t CpuClockHz>
void init_logger_lpuart(
	USART_TypeDef* const usart,
	const stm32_lib::gpio::pin_t& pin_tx,
	uint8_t af
)
{
	usart->CR1 = 0; // ensure UE flag is reset
	stm32_lib::gpio::set_mode_af_lowspeed_pu(pin_tx, af);
	constexpr auto presc_brr = impl::lpuart_presc_brr<CpuClockHz>();
	usart->PRESC = presc_brr.first;
	usart->BRR = presc_brr.second;
	usart->CR1 = impl::cr1;
	usart->CR1 = impl::cr1_en;
}
#endif


// send zero-terminated string
template <typename Usart>
void send(Usart* const usart, const char* s)
{
#ifdef TARGET_NRF52DK
	if (!*s) {
		return;
	}

	usart->TASKS_STARTTX = 1;

	// tx_byte
	usart->EVENTS_TXDRDY = 0;
	usart->TXD = *s;
	++s;
#endif

	while (*s) {
#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7 || defined TARGET_STM32WB55 || defined TARGET_STM32WL55 || defined TARGET_STM32G030 || defined TARGET_STM32G031
		while (! (usart->ISR & USART_ISR_TXE_TXFNF)) {}
		usart->TDR = *s;
#elif defined TARGET_STM32L072
		while (! (usart->ISR & USART_ISR_TXE)) {}
		usart->TDR = *s;
#elif defined TARGET_NRF52DK
		while (! (usart->EVENTS_TXDRDY)) {}
		usart->EVENTS_TXDRDY = 0;
		usart->TXD = *s;
#else
		while (! (usart->SR & USART_SR_TXE)) {}
		usart->DR = *s;
#endif
		++s;
	}

#ifdef TARGET_NRF52DK
	while (! (usart->EVENTS_TXDRDY)) {}
	usart->TASKS_STOPTX = 1;
#endif
}


} // namespace usart


namespace i2c {


inline
void init_pins(
		const gpio::gpio_pin_t& scl, uint32_t af_scl,
		const gpio::gpio_pin_t& sda, uint32_t af_sda
	)
{
	using namespace gpio;
	scl.set(af_t(af_scl), otype_t::open_drain, pupd_t::pu, speed_t::bits_01);
	sda.set(af_t(af_sda), otype_t::open_drain, pupd_t::pu, speed_t::bits_01);
}


inline
void write(I2C_TypeDef* const i2c, uint16_t addr, const uint8_t* const data, size_t size)
{
	//static_assert(addr < 0x80);
	constexpr size_t chunk_max = 255;
	size_t tx_done = 0;
	uint32_t cr2 = 0; // i2c->CR2;
	while (tx_done < size) {
		const size_t tx_rem = size - tx_done;
		const bool last_chunk = (tx_rem <= chunk_max);
		size_t chunk_rem = std::min(tx_rem, chunk_max);

		if (tx_done == 0) {
			// first write to CR2, init
			cr2 = (cr2 & ~(I2C_CR2_SADD_Msk | I2C_CR2_RD_WRN))
				| ((addr << 1) << I2C_CR2_SADD_Pos)
				| I2C_CR2_START_Msk
				;
		}
		cr2 = (cr2 & ~(I2C_CR2_NBYTES_Msk | I2C_CR2_RELOAD_Msk | I2C_CR2_AUTOEND_Msk))
			| (chunk_rem << I2C_CR2_NBYTES_Pos)
			| (last_chunk ? I2C_CR2_AUTOEND_Msk : I2C_CR2_RELOAD_Msk)
			;
		i2c->CR2 = cr2;

		while (chunk_rem > 0) {
			while (!(i2c->ISR & I2C_ISR_TXE)) {} // TODO timeout
			i2c->TXDR = data[tx_done];
			++tx_done;
			--chunk_rem;
		}
	}
	while (i2c->ISR & I2C_ISR_BUSY) {}
}


#if 0
inline
void read(I2C_TypeDef* const i2c, uint16_t addr, uint8_t* data, uint16_t size)
{
	uint16_t rx_done = 0;
	i2c->CR2 = (i2c->CR2 & ~(I2C_CR2_SADD_Msk | I2C_CR2_NBYTES_Msk))
		| I2C_CR2_AUTOEND_Msk
		| I2C_CR2_START_Msk
		| (((addr << 1) | 1) << I2C_CR2_SADD_Pos) // read
		| (size << I2C_CR2_NBYTES_Pos);
	while (rx_done < size) {
		while (!(i2c->ISR & I2C_ISR_RXNE)) {} // TODO timeout
		data[rx_done] = i2c->RXDR;
		++rx_done;
	}
}
#endif


} // namespace i2c


namespace dma {


enum dma_result_t : uint32_t {
	tc = (1 << 0),
	te = (1 << 1),
};


namespace impl {
#if defined(TARGET_STM32H745_CM7) || defined(TARGET_STM32H745_CM4)
	//using dma_channel_t = BDMA_Channel_TypeDef;
	using dma_channel_t = DMA_Stream_TypeDef;
#else
	using dma_channel_t = DMA_Channel_TypeDef;
#endif
}

namespace cast_convert {
	template <uint32_t DmamuxChannelBase> constexpr uint32_t dmamux_to_dmachannel();
#if defined(TARGET_STM32WL55) || defined(TARGET_STM32WB55)
	template<> inline constexpr uint32_t dmamux_to_dmachannel<DMAMUX1_Channel0_BASE>() { return DMA1_Channel1_BASE; }
	template<> inline constexpr uint32_t dmamux_to_dmachannel<DMAMUX1_Channel1_BASE>() { return DMA1_Channel2_BASE; }
	template<> inline constexpr uint32_t dmamux_to_dmachannel<DMAMUX1_Channel2_BASE>() { return DMA1_Channel3_BASE; }
	template<> inline constexpr uint32_t dmamux_to_dmachannel<DMAMUX1_Channel7_BASE>() { return DMA2_Channel1_BASE; }
	template<> inline constexpr uint32_t dmamux_to_dmachannel<DMAMUX1_Channel8_BASE>() { return DMA2_Channel2_BASE; }
	template<> inline constexpr uint32_t dmamux_to_dmachannel<DMAMUX1_Channel9_BASE>() { return DMA2_Channel3_BASE; }
	template<> inline constexpr uint32_t dmamux_to_dmachannel<DMAMUX1_Channel10_BASE>() { return DMA2_Channel4_BASE; }
	template<> inline constexpr uint32_t dmamux_to_dmachannel<DMAMUX1_Channel11_BASE>() { return DMA2_Channel5_BASE; }
#endif
#if defined(TARGET_STM32H745_CM7) || defined(TARGET_STM32H745_CM4)
	template<> inline constexpr uint32_t dmamux_to_dmachannel<DMAMUX1_Channel0_BASE>() { return DMA1_Stream0_BASE; }
	template<> inline constexpr uint32_t dmamux_to_dmachannel<DMAMUX1_Channel1_BASE>() { return DMA1_Stream1_BASE; }
#endif
}

namespace cast {
	inline impl::dma_channel_t* dma_channel(uint32_t DmaChannelBase)
	{
		return reinterpret_cast<impl::dma_channel_t*>(DmaChannelBase);
	}
#if defined(TARGET_STM32WL55) || defined(TARGET_STM32WB55) || defined(TARGET_STM32H745_CM7) || defined(TARGET_STM32H745_CM4)
	inline DMAMUX_Channel_TypeDef* dmamux_channel(uint32_t DmamuxChannelBase)
	{
		return reinterpret_cast<DMAMUX_Channel_TypeDef*>(DmamuxChannelBase);
	}
#endif
}

namespace consts {
	template <uint32_t PeriphBase> constexpr uint32_t dmamux_reqid_tx();
	template <uint32_t PeriphBase> constexpr uint32_t dmamux_reqid_rx();
#if defined(TARGET_STM32WL55)
	template<> constexpr inline uint32_t dmamux_reqid_tx<SUBGHZSPI_BASE>() { return 42; }
	template<> constexpr inline uint32_t dmamux_reqid_rx<SUBGHZSPI_BASE>() { return 41; }
	template<> constexpr inline uint32_t dmamux_reqid_tx<SPI1_BASE>() { return 8; }
	template<> constexpr inline uint32_t dmamux_reqid_rx<SPI1_BASE>() { return 7; }
	template<> constexpr inline uint32_t dmamux_reqid_tx<LPUART1_BASE>() { return 22; }
#endif
#if defined(TARGET_STM32WB55)
	template<> constexpr inline uint32_t dmamux_reqid_tx<USART1_BASE>() { return 15; }
#endif
#if defined(TARGET_STM32H745_CM7) || defined(TARGET_STM32H745_CM4)
	// NOTE: constants are for DMAMUX1 (not for DMAMUX2)
	template<> constexpr inline uint32_t dmamux_reqid_tx<USART3_BASE>() { return 46; }
#endif
}

#if !defined(TARGET_STM32H745_CM7) && !defined(TARGET_STM32H745_CM4)
template <uint32_t SpiBase, uint32_t DmaChannelBaseTx, uint32_t DmaChannelBaseRx>
struct spi_dma_t {
	static void init()
	{
		init_channel<DmaChannelBaseTx>();
		init_channel<DmaChannelBaseRx>();
	}

	static void start(size_t size, const uint8_t* tx_data, uint8_t* rx_data)
	{
		auto const s = spi();
		auto cr2 = s->CR2;

		// RX
		s->CR2 = (cr2 |= SPI_CR2_RXDMAEN);
		{
			auto const ccr_rx = prepare_data<DmaChannelBaseRx>(size, rx_data, dma_ccr_rx);
			auto const dc = cast::dma_channel(DmaChannelBaseRx);
			dc->CCR = ccr_rx;
			dc->CCR = ccr_rx | DMA_CCR_EN;
		}

		// TX
		{
			auto const ccr_tx = prepare_data<DmaChannelBaseTx>(size, const_cast<uint8_t*>(tx_data), dma_ccr_tx);
			auto const dc = cast::dma_channel(DmaChannelBaseTx);
			dc->CCR = ccr_tx;
			dc->CCR = ccr_tx | DMA_CCR_EN;
		}
		s->CR2 = (cr2 |= SPI_CR2_TXDMAEN);

		s->CR1 |= SPI_CR1_SPE;
	}

	static void stop()
	{
		cast::dma_channel(DmaChannelBaseTx)->CCR = 0;
		cast::dma_channel(DmaChannelBaseRx)->CCR = 0;

		// SPI disabling procedure, see datasheet/SPI
		auto const s = spi();
		auto sr = s->SR;

#if defined(TARGET_STM32WL55) || defined(TARGET_STM32G030) || defined(TARGET_STM32G031) /* TODO G03x untested */
		while (sr & SPI_SR_FTLVL_Msk) {
			sr = s->SR;
		}
		while (sr & SPI_SR_BSY) {
			sr = s->SR;
		}
		s->CR1 &= ~SPI_CR1_SPE;
		while (sr & SPI_SR_FRLVL_Msk) {
			sr = s->SR;
		}
#elif defined(TARGET_STM32L072)
		//TODO
		//while (!(sr & SPI_SR_RXNE)) {
		//	sr = s->SR;
		//}
		while (!(sr & SPI_SR_TXE)) {
			sr = s->SR;
		}
		while (sr & SPI_SR_BSY) {
			sr = s->SR;
		}
		s->CR1 &= ~SPI_CR1_SPE;
#else
#error TODO
#endif
		s->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
	}

private:
	static constexpr uint32_t dma_ccr_common_ = (0b01 << DMA_CCR_PL_Pos);
	static constexpr uint32_t dma_ccr_rx = dma_ccr_common_ | DMA_CCR_TCIE | DMA_CCR_TEIE; // enable interrupts for RX only
	static constexpr uint32_t dma_ccr_tx = dma_ccr_common_ | DMA_CCR_DIR;

	static SPI_TypeDef* spi() { return reinterpret_cast<SPI_TypeDef*>(SpiBase); }

	template <uint32_t DmaChannelBase>
	static void init_channel()
	{
		auto const dc = cast::dma_channel(DmaChannelBase);
		dc->CCR = 0;
		dc->CPAR = reinterpret_cast<uint32_t>(&spi()->DR);
	}

	template <uint32_t DmaChannelBase>
	static uint32_t prepare_data(size_t size, uint8_t* data, uint32_t ccr)
	{
		auto const dc = cast::dma_channel(DmaChannelBase);
		dc->CNDTR = size;
		if (data) {
			dc->CMAR = reinterpret_cast<uint32_t>(data);
			ccr |= DMA_CCR_MINC;
		} else {
			// buffer pointer is null (unused)
			// use 1-byte buffer and disable MINC
			static uint8_t unused_data;
			dc->CMAR = reinterpret_cast<uint32_t>(&unused_data);
			ccr &= ~DMA_CCR_MINC;
		}
		return ccr;
	}
};

#if defined(TARGET_STM32WL55)
namespace impl {
	template <uint32_t SpiBase, uint32_t DmamuxChannelBaseTx, uint32_t DmamuxChannelBaseRx>
	using spi_dmamux_base_t = spi_dma_t<
		SpiBase,
		cast_convert::dmamux_to_dmachannel<DmamuxChannelBaseTx>(),
		cast_convert::dmamux_to_dmachannel<DmamuxChannelBaseRx>()
	>;
}

template <uint32_t SpiBase, uint32_t DmamuxChannelBaseTx, uint32_t DmamuxChannelBaseRx>
struct spi_dmamux_t : public impl::spi_dmamux_base_t<SpiBase, DmamuxChannelBaseTx, DmamuxChannelBaseRx> {
	static void init()
	{
		base_t::init();
		cast::dmamux_channel(DmamuxChannelBaseTx)->CCR = (consts::dmamux_reqid_tx<SpiBase>() << DMAMUX_CxCR_DMAREQ_ID_Pos);
		cast::dmamux_channel(DmamuxChannelBaseRx)->CCR = (consts::dmamux_reqid_rx<SpiBase>() << DMAMUX_CxCR_DMAREQ_ID_Pos);
	}
private:
	using base_t = impl::spi_dmamux_base_t<SpiBase, DmamuxChannelBaseTx, DmamuxChannelBaseRx>;
};
#endif
#endif


template <uint32_t UsartBase, uint32_t DmaChannelBaseTx>
struct dev_usart_dma_t {
	static void init()
	{
		auto const dc = dma_channel();
#if defined(TARGET_STM32H745_CM7) || defined(TARGET_STM32H745_CM4)
		dc->CR = dma_scr_tx;
		dc->PAR = reinterpret_cast<uint32_t>(&usart()->TDR);
#else
		dc->CCR = dma_ccr_tx;
		dc->CPAR = reinterpret_cast<uint32_t>(&usart()->TDR);
#endif
	}

	static void start(size_t size, const uint8_t* tx_data)
	{
		auto const dc = dma_channel();
#if defined(TARGET_STM32H745_CM7) || defined(TARGET_STM32H745_CM4)
		dc->M0AR = reinterpret_cast<uint32_t>(tx_data);
		dc->NDTR = size;
		dc->CR = dma_scr_tx_en;
#else
		dc->CMAR = reinterpret_cast<uint32_t>(tx_data);
		dc->CNDTR = size;
		dc->CCR = dma_ccr_tx_en;
#endif
		usart()->CR3 = USART_CR3_DMAT;
	}

	static void stop()
	{
		usart()->CR3 = 0;
#if defined(TARGET_STM32H745_CM7) || defined(TARGET_STM32H745_CM4)
		dma_channel()->CR = dma_scr_tx;
#else
		dma_channel()->CCR = dma_ccr_tx;
#endif
	}

private:
	static USART_TypeDef* usart() { return reinterpret_cast<USART_TypeDef*>(UsartBase); }
	static impl::dma_channel_t* dma_channel() { return cast::dma_channel(DmaChannelBaseTx); }
#if defined(TARGET_STM32H745_CM7) || defined(TARGET_STM32H745_CM4)
	static constexpr uint32_t dma_scr_tx = DMA_SxCR_TRBUFF | (0b00 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | (0b01 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
	static constexpr uint32_t dma_scr_tx_en = dma_scr_tx | DMA_SxCR_EN;
#else
	static constexpr uint32_t dma_ccr_tx = (0b00 << DMA_CCR_PL_Pos) | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE;
	static constexpr uint32_t dma_ccr_tx_en = dma_ccr_tx | DMA_CCR_EN;
#endif
};


#if defined(TARGET_STM32WL55) || defined(TARGET_STM32WB55) || defined(TARGET_STM32H745_CM7) || defined(TARGET_STM32H745_CM4)
namespace impl {
	template <uint32_t UsartBase, uint32_t DmamuxChannelBase>
	using dev_usart_dmamux_base_t = dev_usart_dma_t<UsartBase, cast_convert::dmamux_to_dmachannel<DmamuxChannelBase>()>;
}

template <uint32_t UsartBase, uint32_t DmamuxChannelBaseTx>
struct dev_usart_dmamux_t : public impl::dev_usart_dmamux_base_t<UsartBase, DmamuxChannelBaseTx> {
	static void init()
	{
		base_t::init();
		cast::dmamux_channel(DmamuxChannelBaseTx)->CCR = consts::dmamux_reqid_tx<UsartBase>() << DMAMUX_CxCR_DMAREQ_ID_Pos;
	}
private:
	using base_t = impl::dev_usart_dmamux_base_t<UsartBase, DmamuxChannelBaseTx>;
};
#endif


} // namespace dma


namespace spi {

inline
#ifdef TARGET_STM32F103
void init_pins(
		const gpio::gpio_pin_t& mosi,
		const gpio::gpio_pin_t& miso,
		const gpio::gpio_pin_t& sck
	)
{
	using namespace gpio;
	mosi.set(mode_t::output_2mhz, cnf_t::output_af_pushpull);
	miso.set(mode_t::input, cnf_t::input_float);
	sck.set(mode_t::output_2mhz, cnf_t::output_af_pushpull);
}
#else
void init_pins(
		const gpio::gpio_pin_t& mosi, uint32_t af_mosi,
		const gpio::gpio_pin_t& miso, uint32_t af_miso,
		const gpio::gpio_pin_t& sck, uint32_t af_sck
	)
{
	using namespace gpio;
	mosi.set(af_t(af_mosi), otype_t::push_pull, pupd_t::pd, speed_t::bits_00);
	miso.set(af_t(af_miso), otype_t::push_pull, pupd_t::pd, speed_t::bits_00);
	sck.set(af_t(af_sck), otype_t::push_pull, pupd_t::pd, speed_t::bits_00);
}


inline
void init_pin_nss(const gpio::gpio_pin_t& pin)
{
	using namespace gpio;
	pin.set(mode_t::output, otype_t::push_pull, pupd_t::pu, speed_t::bits_00);
	set_state(pin, 1);
}

#endif


template <typename T>
void write(SPI_TypeDef* const spi, size_t const size, const T* const tx_buf, T* const rx_buf)
{
	static_assert(sizeof(T) == 1 || sizeof(T) == 2); // FIXME
#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
	auto const txdr = reinterpret_cast<volatile T*>(&spi->TXDR);
	auto const rxdr = reinterpret_cast<volatile T*>(&spi->RXDR);
	bool tx_started = false;
#else
	auto const txdr = reinterpret_cast<volatile T*>(&spi->DR);
	auto const rxdr = reinterpret_cast<volatile T*>(&spi->DR);
#endif

	size_t tx_done = 0, rx_done = 0;
	bool mode_tx = true;
	constexpr size_t max_count = 1000000;
	size_t count_without_progress = 0; // count iterations without progress

	while (tx_done < size || rx_done < size) {
		bool progress = false; // did something in current iteration?

#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
		while (tx_done < size && (spi->SR & SPI_SR_TXP)) // fill tx buffer
#else
		if (mode_tx && tx_done < size && (spi->SR & SPI_SR_TXE)) // write one item to tx buffer
#endif
		{
			*txdr = (tx_buf ? tx_buf[tx_done] : 0);
			++tx_done;
			mode_tx = false;
			progress = true;
		}
#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
		if (!tx_started && tx_done>0) {
			// something was written to tx buffer, start transfer
			spi->CR1 |= SPI_CR1_CSTART_Msk;
			tx_started = true;
		}
#endif

#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
		while (rx_done < size && (spi->SR & SPI_SR_RXP))
#else
		if (!mode_tx && rx_done < size && (spi->SR & SPI_SR_RXNE))
#endif
		{
			const T d = *rxdr;
			if (rx_buf) {
				rx_buf[rx_done] = d;
			}
			++rx_done;
			mode_tx = true;
			progress = true;
		}

		if (progress) {
			count_without_progress = 0; // done something, reset counter
		} else {
			++count_without_progress;
			if (count_without_progress >= max_count) {
				// TODO handle timeout
			}
		}
	}

#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
#else
	count_without_progress = 0;
	while(spi->SR & SPI_SR_BSY) {
		++count_without_progress;
		if (count_without_progress >= max_count) {
			// TODO handle timeout
		}
	}
#endif
}

template <typename T>
T write(SPI_TypeDef* const spi, T data)
{
	T rx_buf = 0;
	write(spi, 1, &data, &rx_buf);
	return rx_buf;
}


} // namespace spi


#if defined TARGET_STM32WL55
namespace ipcc {


template <uint8_t Ch>
void c1_mask_rx_int()
{
	IPCC->C1MR |= uint32_t(1 << Ch);
}

template <uint8_t Ch>
void c1_unmask_rx_int()
{
	IPCC->C1MR &= ~(uint32_t(1 << Ch));
}

template <uint8_t Ch>
bool c2_to_c1_tx_is_free()
{
	return (IPCC->C2TOC1SR & (1 << Ch)) == 0;
}

template <uint8_t Ch>
void c2_to_c1_send()
{
	IPCC->C2SCR = (1 << (Ch + 16));
}

template <uint8_t Ch>
void c1_mark_received()
{
	IPCC->C1SCR = (1 << Ch);
}


} // ipcc
#endif


#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
namespace hsem {


template <size_t N>
struct hsem_t {
	static_assert(N >= HSEM_SEMID_MIN);
	static_assert(N <= HSEM_SEMID_MAX);
	bool fast_take() // return false (to match with HAL_OK) on successful lock
	{
		if (HSEM->RLR[N] == (HSEM_CR_COREID_CURRENT | HSEM_RLR_LOCK)) {
			return false; // taken successfully
		}
		return true;
	}

	void release()
	{
		HSEM->R[N] = HSEM_CR_COREID_CURRENT;
	}
};


} // namespace hsem
#endif

} // namespace stm32_lib


#endif

