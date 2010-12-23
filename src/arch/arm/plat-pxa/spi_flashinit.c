#if defined(CONFIG_MTD_M25P80) && defined(CONFIG_SPI_PXA2XX)
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <linux/spi/flash.h>
#include <linux/mtd/partitions.h>
#include <plat/pxa2xx_spi.h>
#include <plat/part_table.h>
#include <linux/spi/spi.h>

DECLARE_ASPENITE_NOR_PARTITIONS(aspenite_nor_partitions);

static struct pxa2xx_spi_chip m25pxx_spi_info = {
	.tx_threshold = 1,
	.rx_threshold = 1,
	.timeout = 1000,
	.gpio_cs = 110
};
static const struct flash_platform_data pxa2xx_spi_data = {
	.name		= "pxa2xx_spi_nor",
	.type		= "mx25l64",
	.parts		= aspenite_nor_partitions,
	.nr_parts	= ARRAY_SIZE(aspenite_nor_partitions),
};

static struct spi_board_info __initdata m25pxx_spi_board_info[] = {
	{
		.modalias = "m25p80",
		.mode = SPI_MODE_0,
		.max_speed_hz = 260000,
		.bus_num = 2,
		.chip_select = 0,
		.platform_data = &pxa2xx_spi_data,
		.controller_data = &m25pxx_spi_info,
		.irq = -1,
	},
};

void spi_flashinit(void)
{
	spi_register_board_info(m25pxx_spi_board_info,
				ARRAY_SIZE(m25pxx_spi_board_info));
}
#endif
