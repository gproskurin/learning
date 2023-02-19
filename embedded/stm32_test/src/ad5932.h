#ifndef _gpr_ad5932_h_included_
#define _gpr_ad5932_h_included_


class ad5932_t {
	spi_t spi_;
	GPIO_TypeDef* const mclk_gpio_;
	int const mclk_pin_;
public:
	ad5932_t();
	void start();
private:
	void ctrl_pulse();
};


#endif

