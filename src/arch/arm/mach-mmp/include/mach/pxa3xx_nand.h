#ifndef __ASM_ARCH_PXA3XX_NAND_H
#define __ASM_ARCH_PXA3XX_NAND_H

#ifdef   __KERNEL__
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#endif

#define NUM_CHIP_SELECT		2
#define CMD_POLL_SIZE		5

struct pxa3xx_nand_timing {
	unsigned int		tCH;  /* Enable signal hold time */
	unsigned int		tCS;  /* Enable signal setup time */
	unsigned int		tWH;  /* ND_nWE high duration */
	unsigned int		tWP;  /* ND_nWE pulse time */
	unsigned int		tRH;  /* ND_nRE high duration */
	unsigned int		tRP;  /* ND_nRE pulse width */
	unsigned int		tR;   /* ND_nWE high to ND_nRE low for read */
	unsigned int		tWHR; /* ND_nWE high to ND_nRE low for status read */
	unsigned int		tAR;  /* ND_ALE low to ND_nRE low delay */
};

struct pxa3xx_nand_cmdset {
	uint16_t        	read1;
	uint16_t        	read2;
	uint16_t        	program;
	uint16_t        	read_status;
	uint16_t        	read_id;
	uint16_t        	erase;
	uint16_t        	reset;
	uint16_t        	lock;
	uint16_t       		unlock;
	uint16_t        	lock_status;
};

struct pxa3xx_nand_flash {
	const struct pxa3xx_nand_timing *timing; /* NAND Flash timing */
	const struct pxa3xx_nand_cmdset *cmdset;
	const char name[18];

	uint32_t 		page_per_block;	/* Pages per block (PG_PER_BLK) */
	uint32_t 		page_size;	/* Page size in bytes (PAGE_SZ) */
	uint32_t 		flash_width;	/* Width of Flash memory (DWIDTH_M) */
	uint32_t 		dfc_width;	/* Width of flash controller(DWIDTH_C) */
	uint32_t 		num_blocks;	/* Number of physical blocks in Flash */
	uint32_t 		chip_id;
	uint32_t		ecc_type;	/* 0 for Hamming, 1 for BCH */
};




struct pxa3xx_nand_platform_data {

	/* the data flash bus is shared between the Static Memory
	 * Controller and the Data Flash Controller,  the arbiter
	 * controls the ownership of the bus
	 */
	int			enable_arbiter;
	int			use_dma;	/* use DMA ? */
	int			RD_CNT_DEL;

	struct mtd_partition    *parts[NUM_CHIP_SELECT];
	unsigned int            nr_parts[NUM_CHIP_SELECT];
};

extern void pxa3xx_set_nand_info(struct pxa3xx_nand_platform_data *info);

/* registers and bit definitions */
#define NDCR			(0x00) /* Control register */
#define NDTR0CS0		(0x04) /* Timing Parameter 0 for CS0 */
#define NDTR1CS0		(0x0C) /* Timing Parameter 1 for CS0 */
#define NDSR			(0x14) /* Status Register */
#define NDPCR			(0x18) /* Page Count Register */
#define NDBBR0			(0x1C) /* Bad Block Register 0 */
#define NDBBR1			(0x20) /* Bad Block Register 1 */
#define NDREDEL			(0x24) /* Read Enable Return Delay Register */
#define NDECCCTRL		(0x28) /* ECC Control Register */
#define NDBZCNT			(0x2C) /* Timer for NDRnB0 and NDRnB1 */
#define NDMUTEX			(0x30) /* Mutex Lock Register */
#define NDCMDMAT0		(0x34) /* Partition Command Match Register 0 */
#define NDCMDMAT1		(0x38) /* Partition Command Match Register 1 */
#define NDCMDMAT2		(0x3C) /* Partition Command Match Register 2 */
#define NDDB			(0x40) /* Data Buffer */
#define NDCB0			(0x48) /* Command Buffer0 */
#define NDCB1			(0x4C) /* Command Buffer1 */
#define NDCB2			(0x50) /* Command Buffer2 */
#define NDCB3			(0x50) /* Command Buffer3 */
#define NDARBCR			(0x5C) /* DFI Arbitration Control Register */
#define NDPTXCS0		(0x60) /* Partition Region Control Register 0 */
#define NDPTXCS1		(0x64) /* Partition Region Control Register 1 */
#define NDPTXCS2		(0x68) /* Partition Region Control Register 2 */
#define NDPTXCS3		(0x6C) /* Partition Region Control Register 3 */
#define NDPTXCS4		(0x70) /* Partition Region Control Register 4 */
#define NDPTXCS5		(0x74) /* Partition Region Control Register 5 */
#define NDPTXCS6		(0x78) /* Partition Region Control Register 6 */
#define NDPTXCS7		(0x7C) /* Partition Region Control Register 7 */

/* NDCR Register */
#define NDCR_SPARE_EN		(0x1 << 31)
#define NDCR_ECC_EN		(0x1 << 30)
#define NDCR_DMA_EN		(0x1 << 29)
#define NDCR_ND_RUN		(0x1 << 28)
#define NDCR_DWIDTH_C		(0x1 << 27)
#define NDCR_DWIDTH_M		(0x1 << 26)
#define NDCR_PAGE_SZ_MASK	(0x3 << 24)
#define NDCR_PAGE_SZ(x)		(((x) << 24) & NDCR_PAGE_SZ_MASK)
#define NDCR_SEQ_DIS		(0x1 << 23)
#define NDCR_ND_STOP		(0x1 << 22)
#define NDCR_FORCE_CSX		(0x1 << 21)
#define NDCR_CLR_PG_CNT		(0x1 << 20)
#define NDCR_STOP_ON_UNCOR	(0x1 << 19)
#define NDCR_RD_ID_CNT_MASK	(0x7 << 16)
#define NDCR_RD_ID_CNT(x)	(((x) << 16) & NDCR_RD_ID_CNT_MASK)

#define NDCR_RA_START		(0x1 << 15)
#define NDCR_PG_PER_BLK_MASK	(0x3 << 13)
#define NDCR_PG_PER_BLK(x)	(((x) << 13) & NDCR_PG_PER_BLK_MASK)
#define NDCR_ND_ARB_EN		(0x1 << 12)
#define NDCR_RDYM		(0x1 << 11)
#define NDCR_CS0_PAGEDM		(0x1 << 10)
#define NDCR_CS1_PAGEDM		(0x1 << 9)
#define NDCR_CS0_CMDDM		(0x1 << 8)
#define NDCR_CS1_CMDDM		(0x1 << 7)
#define NDCR_CS0_BBDM		(0x1 << 6)
#define NDCR_CS1_BBDM		(0x1 << 5)
#define NDCR_UNCERRM		(0x1 << 4)
#define NDCR_CORERRM		(0x1 << 3)
#define NDCR_WRDREQM		(0x1 << 2)
#define NDCR_RDDREQM		(0x1 << 1)
#define NDCR_WRCMDREQM		(0x1)
#define NDCR_INT_MASK		(0xFFF)

/* Data Controller Timing Paramter x Register For CSx */
#define NDTR0_SELCNTR		(0x1 << 26)
#define NDTR0_RD_CNT_DEL_MASK	(0xF << 22)
#define NDTR0_RD_CNT_DEL(x)	(((x) << 22) & NDTR0_RD_CNT_DEL_MASK)
#define NDTR0_tCH(c)		(min((c), 7) << 19)
#define NDTR0_tCS(c)		(min((c), 7) << 16)
#define NDTR0_tWH(c)		(min((c), 7) << 11)
#define NDTR0_tWP(c)		(min((c), 7) << 8)
#define NDTR0_ETRP		(0x1 << 6)
#define NDTR0_tRH(c)		(min((c), 7) << 3)
#define NDTR0_tRP(c)		(min((c), 7) << 0)

#define NDTR1_tR(c)		(min((c), 65535) << 16)
#define NDTR1_WAIT_MODE		(0x1 << 15)
#define NDTR1_tWHR(c)		(min((c), 15) << 4)
#define NDTR1_tAR(c)		(min((c), 15) << 0)

/* NDSR Register */
#define NDSR_ERR_CNT_MASK	(0x1F << 16)
#define NDSR_ERR_CNT(x)		(((x) << 16) & NDSR_ERR_CNT_MASK)
#define NDSR_TRUSTVIO		(0x1 << 15)
#define NDSR_MASK		(0xFFFF)
#define NDSR_RDY		(0x1 << 12)		
#define NDSR_FLASH_RDY		(0x1 << 11)
#define NDSR_CS0_PAGED		(0x1 << 10)
#define NDSR_CS1_PAGED		(0x1 << 9)
#define NDSR_CS0_CMDD		(0x1 << 8)
#define NDSR_CS1_CMDD		(0x1 << 7)
#define NDSR_CS0_BBD		(0x1 << 6)
#define NDSR_CS1_BBD		(0x1 << 5)
#define NDSR_UNCERR		(0x1 << 4)
#define NDSR_CORERR		(0x1 << 3)
#define NDSR_WRDREQ		(0x1 << 2)
#define NDSR_RDDREQ		(0x1 << 1)
#define NDSR_WRCMDREQ		(0x1)

/* NDPCR Register */
#define NDPCR_PG_CNT_1_MASK	(0xFF << 16)
#define NDPCR_PG_CNT_1(x)	(((x) << 16) & NDPCR_PG_CNT_1_MASK)
#define NDPCR_PG_CNT_0_MASK	(0xFF)
#define NDPCR_PG_CNT_0(x)	((x) & NDPCR_PG_CNT_0_MASK)

/* READ Enable Return Delay Register */
#define NDREDEL_ND_DIN_SEL	(0x1 << 25)
#define NDREDEL_ND_DATA_D_MASK	(0x3 << 8)
#define NDREDEL_ND_DATA_DLY(x)	(((x) << 8) & NDREDEL_ND_DATA_D_MASK)
#define NDREDEL_ND_RECLK_D_MASK	(0xF << 4)
#define NDREDEL_ND_RECLK_DLY(x)	(((x) << 4) & NDREDEL_ND_RECLK_D_MASK)
#define NDREDEL_ND_RE_D_MASK	(0xF)
#define NDREDEL_ND_RE_DLY(x)	((x) & NDREDEL_ND_RE_D_MASK)

/* ECC Control Register */
#define NDECCCTRL_ECC_SPARE_MSK	(0xFF << 7)
#define NDECCCTRL_ECC_SPARE(x)	(((x) << 7) & NDECCCTRL_ECC_SPARE_MSK)
#define NDECCCTRL_ECC_THR_MSK	(0x3F << 1)
#define NDECCCTRL_ECC_THRESH(x)	(((x) << 1) & NDECCCTRL_ECC_THR_MSK)
#define NDECCCTRL_BCH_EN	(0x1)

/* Timer for ND_RnBx */
#define NDBZCNT_MASK		(0xFFFF)
#define NDBZCNT_ND_RNB_CNT1(x)	(((x & NDBZCNT_MASK) << 16)
#define NDBZCNT_ND_RNB_CNT0(x)	(x & NDBZCNT_MASK)

		/* NAND Controller MUTEX Lock Register */
#define NDMUTEX_MUTEX		(0x1)

		/* Partition Command Match Registers */
#define NDCMDMAT_VALIDCNT_MASK	(0x3)
#define NDCMDMAT_CMD_MASK	(0xFF)
#define NDCMDMAT_VALIDCNT	((x & NDCMDMAT_VALIDCNT_MASK) << 30)
#define NDCMDMAT_NAKEDDIS2	(0x1 << 29)
#define NDCMDMAT_ROWADD2	(0x1 << 28)
#define NDCMDMAT_CMD2(x)	((x & NDCMDMAT) << 20)
#define NDCMDMAT_NAKEDDIS1	(0x1 << 29)
#define NDCMDMAT_ROWADD1	(0x1 << 28)
#define NDCMDMAT_CMD1(x)	((x & NDCMDMAT) << 20)
#define NDCMDMAT_NAKEDDIS0	(0x1 << 29)
#define NDCMDMAT_ROWADD0	(0x1 << 28)
#define NDCMDMAT_CMD0(x)	((x & NDCMDMAT) << 20)

		/* NAND Controller Command Buffers */
#define NDCB0_CMD_XTYPE_MASK	(0x7 << 29)
#define NDCB0_CMD_XTYPE(x)	(((x) << 29) & NDCB0_CMD_XTYPE_MASK)
#define NDCB0_LEN_OVRD		(0x1 << 28)
#define NDCB0_RDY_BYP		(0x1 << 27)
#define NDCB0_ST_ROW_EN		(0x1 << 26)
#define NDCB0_AUTO_RS		(0x1 << 25)
#define NDCB0_CSEL		(0x1 << 24)
#define NDCB0_CMD_TYPE_MASK	(0x7 << 21)
#define NDCB0_CMD_TYPE(x)	(((x) << 21) & NDCB0_CMD_TYPE_MASK)
#define NDCB0_NC		(0x1 << 20)
#define NDCB0_DBC		(0x1 << 19)
#define NDCB0_ADDR_CYC_MASK	(0x7 << 16)
#define NDCB0_ADDR_CYC(x)	(((x) << 16) & NDCB0_ADDR_CYC_MASK)
#define NDCB0_CMD2_MASK		(0xff << 8)
#define NDCB0_CMD1_MASK		(0xff)

#define NDCB_MASK		(0xFF)
#define NDCB1_ADDR4(x)		((x & NDCB_MASK) << 24)
#define NDCB1_ADDR3(x)		((x & NDCB_MASK) << 16)
#define NDCB1_ADDR2(x)		((x & NDCB_MASK) << 8)
#define NDCB1_ADDR1(x)		(x & NDCB_MASK)

#define NDCB2_ST_MASK(x)	((x & NDCB_MASK) << 24)
#define NDCB2_ST_CMD(x)		((x & NDCB_MASK) << 16)
#define NDCB2_PAGE_COUNT(x)	((x & NDCB_MASK) << 8)
#define NDCB2_ADDR5(x)		(x & NDCB_MASK)

#define NDCB3_ADDR7(x)		((x & NDCB_MASK) << 24)
#define NDCB3_ADDR6(x)		((x & NDCB_MASK) << 16)
#define NDCB3_NDLENCNT_MASK	(0xFFFF)
#define NDCB3_NDLENCNT(x)	(x & NDCB3_NDLENCNT_MASK)

/* DFI Arbitration Control Register */
#define NDARBCR_MASK		(0xFFFF)
#define NDARBCR_ARB_CNT(x)	(x & NDARBCR_MASK)

/* Partition Region Control Registers for CSx */
#define NDPTXCS_VALID		(0x1 << 31)
#define NDPTXCS_LOCK		(0x1 << 30)
#define NDPTXCS_TRUSTED		(0x1 << 29)
#define NDPTXCS_BLOCKADD_MASK	(0xFFFFFF)
#define NDPTXCS_BLOCKADD(x)	((x) & NDPTXCS_BLOCKADD_MASK)

/* dma-able I/O address for the NAND data and commands */
#define NDCB0_DMA_ADDR		(0xd4283048)
#define NDDB_DMA_ADDR		(0xd4283040)

/* macros for registers read/write */
#define nand_writel(nand, off, val)	\
	__raw_writel((val), (nand)->mmio_base + (off))

#define nand_readl(nand, off)		\
	__raw_readl((nand)->mmio_base + (off))

/* error code and state */
enum ecc_type {
	ECC_NONE = 0,
	ECC_HAMMIN,
	ECC_BCH,
};



#endif /* __ASM_ARCH_PXA3XX_NAND_H */
