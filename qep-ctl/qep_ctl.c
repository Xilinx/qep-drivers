/*
 * Copyright (c) 2019, Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file )
 */

/*  qep_ctl.c - An utility to configure QDMA Ethernet Platform (QEP) */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>

#include <sys/mman.h>
#include <sys/fcntl.h>
#include <netinet/ether.h>

/************************Macros**********************************************/

#define PCIE_DEV_ID_MGMT_PF  0x7000
#define DEFAULT_USER_BAR 	2
#define DEFAULT_USER_BAR_LEN  0x4000000

#define MAX_BDF_LEN 	32
#define MIN_BDF_LEN 	7
#define MAX_BUF_LEN 	1024
#define BYTE_LEN 	8

/***************************Registers******************************************/

#define SEC_TX_CFG_W 0x200000
#define SEC_RX_CFG_W 0x203000

#define SEC_META_LOW_W 0x201000
#define SEC_META_HIGH_W 0x202000

#define VLAN_MASK 0xfff
#define TX_VLAN_BIT 20
#define TX_DEI_BIT 19
#define TX_PCP_BIT 16

/****************************************************************************/
enum direction {
	TX_CORE, RX_CORE, BOTH_CORE, CORE_MAX
};

struct vlan_tag {
	uint16_t pcp :3; /* Priority code point */
	uint16_t dei :1; /* Drop eligible indicator */
	uint16_t vid :12; /* VLAN identifier */
};

enum cmd_type {
	OPTION_MAC_ADDR,
	OPTION_VLAN_TAG,
	OPTION_ENABLE_CONF,
	OPTION_USER_BDF,
	WRITE_CONFIG,
	SHOW_CONFIG,
	READ_REG,
	WRITE_REG,
	CMD_MAX
};

struct qepsec_enable {
	uint8_t mac :1;
	uint8_t vlan :1;
	uint8_t discard :1;
	uint8_t disable :1;
};

struct qepctl_cntx {
	uint8_t bar_num;
	uint32_t offset;
	uint32_t val;
	struct ether_addr mac;
	struct vlan_tag vlan;
	char bdf[MAX_BDF_LEN];
	enum direction dir;
	bool cmd[CMD_MAX];
	union {
		uint8_t core_en[CORE_MAX];
		struct qepsec_enable en[CORE_MAX];
	};

};
/****************************************************************************/
static void usage()
{
	const char *app_name = "qep-ctl";
	fprintf(stdout, " ./%s CMD [ options ] \n", app_name);
	fprintf(stdout, "      CMD = [ config | show | read | write | help ] \n");
	fprintf(stdout, "      Options =  -d BDF or dev \n");
	fprintf(stdout, "                 -b bar num \n");
	fprintf(stdout, "                 -c [ tx | rx ]  \n");
	fprintf(stdout, "                 -m mac_addr \n");
	fprintf(stdout, "                 -v VID,DEI,PCP  \n");
	fprintf(stdout, "                 -e [ mac,vlan,discard ] [ disable ]  \n");
	fprintf(stdout, "Examples:\n");
	fprintf(stdout, "  ./%s config -m 01:02:03:03:04:06 -v 4,0,3 -e mac,vlan -c tx -d 18:00.1 \n", app_name);
	fprintf(stdout, "  ./%s read 0x20000 -d 18:00.1 -b 2 \n", app_name);
	fprintf(stdout, "  ./%s write 0x20000 0x12 -d 18:00.1 -b 2 \n", app_name);
	fprintf(stdout, "bdf(-d) is only needed in case of multiple device\n");
}

static uint32_t readl(uint32_t *addr)
{
	return *addr;
}

static void writel(uint32_t val, uint32_t *addr)
{
	*addr = val;
}

static void *mmap_pci_bar(char* bdf, uint8_t bar_num)
{
	int fd;
	void *base_addr = NULL;
	uint32_t bar_len = DEFAULT_USER_BAR_LEN;
	char filename[MAX_BUF_LEN];

	if (!bdf){
		fprintf(stderr, "BDF is NULL \n");
		return NULL;
	}

	sprintf(filename, "/sys/bus/pci/devices/0000:%s/resource%d",
			bdf, bar_num);

	fd = open(filename, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, " %s : Resource file(%s) open failed Error:%s \n",
				__func__, filename, strerror(errno));
		fprintf(stderr, "Invalid BDF or BAR Number ! pass BDF(-d) or Bar(-b) as input \n");
		system("lspci -d 10ee: -v");
		return NULL;
	}

	base_addr = mmap(0, bar_len, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, 0);
	if (base_addr == MAP_FAILED) {
		fprintf(stderr, " %s : mapped failed Error:%s \n", __func__,
				strerror(errno));
		base_addr = NULL;
	}

	close(fd);
	return base_addr;
}

static int bdf_from_devid(uint32_t dev_id, char* bdf_usr, unsigned len)
{
	FILE *fp;
	char cmd[MAX_BUF_LEN] = {'\0'};
	char bdf[MAX_BDF_LEN] = {'\0'};
	char *token;

	sprintf(cmd, "lspci -d 10ee:%x | cut -d \' \' -f1", dev_id);

	fp = popen(cmd, "r");
	if (fp == NULL) {
		fprintf(stderr, "Failed to exec command: %s \n", cmd);
		return -EINVAL;
	}

	if (fgets(bdf, sizeof(bdf) - 1, fp) == NULL) {
		fprintf(stderr, "Failed to run command\n");
		return -EINVAL;
	}

	if (!bdf[0])
		return -EINVAL;

	token = strtok(bdf, " ");
	if (!token)
		return -EINVAL;

	if (strtok(NULL, " ")){
		fprintf(stderr, "Mutiple bdf found !! Specify BDF with -d option \n");
		return -EINVAL;
	}

	if (strlen(bdf) > MIN_BDF_LEN && len > strlen(bdf)) {
		strncpy(bdf_usr, bdf, strlen(bdf)-1);
		return 0;
	}

	fprintf(stdout, "Could not find valid Xilinx device, provide BDF(-d) option \n");
	return -EINVAL;

}

static void writehw_en(void *addr, enum direction dir, struct qepsec_enable *en)
{
	uint32_t offset, en_reg;

	if (dir == TX_CORE)
		offset = SEC_TX_CFG_W;
	else
		offset = SEC_RX_CFG_W;

	en_reg = readl(addr + offset);

	if (en->disable) {
		writel(0, addr + offset);
		return;
	}

	//enable the core
	en_reg |= 1;

	if (en->vlan)
		en_reg |= 1 << 1;
	if (en->mac)
		en_reg |= 1 << 2;
	if (en->discard)
		en_reg |= 1 << 3;

	writel(en_reg, addr + offset);
}

static void readhw(void *addr, enum direction dir, uint8_t *en, uint32_t *rlow,
		uint32_t *rhigh)
{

	*rlow = readl(addr + SEC_META_LOW_W);
	*rhigh = readl(addr + SEC_META_HIGH_W);

	if (dir == TX_CORE) {
		en[dir] = readl(addr + SEC_TX_CFG_W);
	} else if (dir == RX_CORE) {
		en[dir] = readl(addr + SEC_RX_CFG_W);
	} else if (dir == BOTH_CORE) {
		en[TX_CORE] = readl(addr + SEC_TX_CFG_W);
		en[RX_CORE] = readl(addr + SEC_RX_CFG_W);
	} else
		fprintf(stderr, "%s:INVALID CORE TYPE", __FUNCTION__);
}

static void readhw_config(void *addr, struct qepctl_cntx * cntx)
{
	uint32_t rlow;
	uint32_t rhigh;

	readhw(addr, cntx->dir, cntx->core_en, &rlow, &rhigh);

	cntx->mac.ether_addr_octet[0] |= (rhigh & 0xff00) >> BYTE_LEN;
	cntx->mac.ether_addr_octet[1] |= rhigh & 0xff;

	cntx->mac.ether_addr_octet[2] |= (rlow & 0xff000000) >> (3 * BYTE_LEN);
	cntx->mac.ether_addr_octet[3] |= (rlow & 0xff0000) >> (2 * BYTE_LEN);
	cntx->mac.ether_addr_octet[4] |= (rlow & 0xff00) >> BYTE_LEN;
	cntx->mac.ether_addr_octet[5] |= rlow & 0xff;

	cntx->vlan.vid |= (rhigh & 0xfff00000) >> TX_VLAN_BIT;
	cntx->vlan.dei |= (rhigh & 0xf0000) >> TX_DEI_BIT;
	cntx->vlan.pcp |= (rhigh & 0xf0000) >> TX_PCP_BIT;

}

static int qepsec_config_show(void *addr, struct qepctl_cntx * cntx)
{

	if (!addr || !cntx)
		return -EINVAL;

	readhw_config(addr, cntx);

	fprintf(stdout, "SECURITY BLOCK CONFIG\n");
	fprintf(stdout, "---------------------\n");
	fprintf(stdout, "MAC Addr: %s\n", ether_ntoa(&cntx->mac));
	fprintf(stdout, "Vlan ID: %d \nPriority Class:%d \nDrop Eligibility Indicator :%d\n",
			cntx->vlan.vid, cntx->vlan.pcp, cntx->vlan.dei);
	fprintf(stdout, "Enable tx: %d rx: %d\n",
			cntx->core_en[TX_CORE], cntx->core_en[RX_CORE]);
	fprintf(stdout, "---------------------\n");

	return 0;
}

static int qepsec_config(void *addr, struct qepctl_cntx *cntx)
{
	uint32_t low, high;
	uint8_t en[CORE_MAX];
	bool wr_mac = false;

	readhw(addr, cntx->dir, en, &low, &high);

	if (cntx->cmd[OPTION_MAC_ADDR]) {
		high = high & ~0xffff;
		high = high | cntx->mac.ether_addr_octet[0] << BYTE_LEN
				| cntx->mac.ether_addr_octet[1];
		low = cntx->mac.ether_addr_octet[2] << (BYTE_LEN * 3)
				| cntx->mac.ether_addr_octet[3] << (BYTE_LEN * 2) |
				cntx->mac.ether_addr_octet[4] << BYTE_LEN
				| cntx->mac.ether_addr_octet[5];
		wr_mac = true;
	}

	if (cntx->cmd[OPTION_VLAN_TAG]) {
		high &= ~(VLAN_MASK << (TX_VLAN_BIT - 1));
		high |= cntx->vlan.vid << TX_VLAN_BIT;
		wr_mac = true;
		if (cntx->vlan.dei)
			high |= cntx->vlan.dei << TX_DEI_BIT;
		if (cntx->vlan.pcp) {
			high &= ~(0x7 << (TX_PCP_BIT-1));
			high |= cntx->vlan.pcp << TX_PCP_BIT;
		}
	}

	if (wr_mac) {
		writel(low, addr + SEC_META_LOW_W);
		writel(high, addr + SEC_META_HIGH_W);
	}

	if (cntx->cmd[OPTION_ENABLE_CONF]) {
		if (cntx->dir != BOTH_CORE)
			writehw_en(addr, cntx->dir, &cntx->en[cntx->dir]);
		else {
			writehw_en(addr, TX_CORE, &cntx->en[TX_CORE]);
			writehw_en(addr, RX_CORE, &cntx->en[RX_CORE]);
		}
	}

	return 0;
}

static int parse_mac(char *str, struct ether_addr *mac)
{
	int count = 0;
	char str_buf[MAX_BUF_LEN];
	char *token;
	char *end;

	strncpy(str_buf, str, strlen(str));
	token = strtok(str_buf, ":");
	while (token) {
		mac->ether_addr_octet[count++] = strtoul(token, &end, 16);
		if (strlen(end)) {
			fprintf(stderr, "Invalid Octet %s  \n", token);
			return -EINVAL;
		}
		token = strtok(NULL, ":");
	}

	if (count != ETH_ALEN)
		return -EINVAL;

	return 0;
}

static int parse_enable_dir(char *input, enum direction dir,
		struct qepctl_cntx *cntx)
{
	if (strcasecmp(input, "mac") == 0)
		cntx->en[dir].mac = 1;
	else if (strcasecmp(input, "vlan") == 0)
		cntx->en[dir].vlan = 1;
	else if (strcasecmp(input, "discard") == 0)
		cntx->en[dir].discard = 1;
	else if (strcasecmp(input, "disable") == 0)
		cntx->en[dir].disable = 1;
	else {
		fprintf(stderr, "Invalid Enable Argument: %s \n", input);
		return EINVAL;
	}

	return 0;
}

static int parse_enable_multi(char *input, enum direction dir, struct qepctl_cntx *cntx)
{
	char *token;
	char param[MAX_BUF_LEN];
	int ret;

	if (!input)
		return -EINVAL;

	strcpy(param, input);
	token = strtok(param, ",");
	while (token) {
		ret = parse_enable_dir(token, dir, cntx);
		if (ret != 0){
			fprintf(stderr, "Invalid Enable Options: %s \n", token);
			return ret;
		}
		token = strtok(NULL, ",");
	}
	return 0;

}

static int parse_enable(char *input, struct qepctl_cntx *cntx)
{

	int ret;

	if (cntx->dir == BOTH_CORE) {
		ret = parse_enable_multi(input, TX_CORE, cntx);
		if (ret != 0)
			return ret;

		ret = parse_enable_multi(input, RX_CORE, cntx);
		if (ret != 0)
			return ret;
	} else {
		ret = parse_enable_multi(input, cntx->dir, cntx);
		if (ret != 0)
			return ret;
	}

	return 0;
}

static int parse_vlan_tag(char *input, struct qepctl_cntx *cntx)
{
	char *token;
	char param[MAX_BUF_LEN];
	char *end;
	int temp;

	if (!input)
		return -EINVAL;

	strcpy(param, input);

	token = strtok(param, ",");
	if (!token)
		return -EINVAL;

	temp = strtoul(token, &end, 0);
	if (strlen(end)) {
		fprintf(stderr, "Invalid Number Argument : %s \n", token);
		return -EINVAL;
	}
	if (temp < 0 || temp >= 0xfff ){
		fprintf(stderr, "Invalid VLAN ID: %d \n", temp);
		return -EINVAL;
	}
	cntx->vlan.vid = temp;

	token = strtok(NULL, ",");
	if (!token)
		return 0;

	temp = strtoul(token, &end, 0);
	if (strlen(end)) {
		fprintf(stderr, "Invalid Number Argument: %s \n", token);
		return -EINVAL;
	}
	if (temp < 0 || temp > 1){
		fprintf(stderr, "Invalid DEI Value: %d \n", temp);
		return -EINVAL;
	}
	cntx->vlan.dei = temp;


	token = strtok(NULL, ",");
	if (!token)
		return 0;
	temp = strtoul(token, &end, 0);
	if (strlen(end)) {
		fprintf(stderr, "Invalid Number Argument: %s \n", token);
		return -EINVAL;
	}
	if (temp < 0 || temp > 7 ){
		fprintf(stderr, "Invalid PCP Value: %d \n", temp);
		return -EINVAL;
	}
	cntx->vlan.pcp = temp;

	token = strtok(NULL, ",");
	if (token){
		fprintf(stderr, "Invalid , separated value \n");
		return -EINVAL;
	}

	return 0;

}
int parse_options(int argc, char **argv, struct qepctl_cntx *cntx)
{
	int nopt = 0;
	char opt;
	char *end;

	cntx->dir = BOTH_CORE;
		cntx->bar_num = DEFAULT_USER_BAR ;

	while ((opt = getopt(argc, argv, "+e:v:m:c:d:b:h")) != -1) {
		nopt++;
		switch (opt) {
		case 'h':
			usage();
			break;
		case 'd':
			strcpy(&cntx->bdf[0], optarg);
			cntx->cmd[OPTION_USER_BDF] = true;
			break;
		case 'b':
			cntx->bar_num = strtoul(optarg, &end, 0);
			if (strlen(end)) {
				fprintf(stderr, "Invalid Number Argument");
				return -EINVAL;
			}
			break;
		case 'c':
			if (strcasecmp(optarg, "TX") == 0)
				cntx->dir = TX_CORE;
			else if (strcasecmp(optarg, "RX") == 0)
				cntx->dir = RX_CORE;
			else
				cntx->dir = BOTH_CORE;
			break;
		case 'm':
			if (!cntx->cmd[WRITE_CONFIG])
				return -EINVAL;
			if (parse_mac(optarg, &cntx->mac) != 0) {
				fprintf(stderr, "Invalid MAC ADDR Format, Accepted format: 11:22:33:44:55:66 \n");
				return 0;
			}
			cntx->cmd[OPTION_MAC_ADDR] = true;
			break;
		case 'v':
			if (!cntx->cmd[WRITE_CONFIG])
				return -EINVAL;
			if (parse_vlan_tag(optarg, cntx) != 0) {
				fprintf(stderr, "Invalid VLAN option. Valid option are VID[,DEI[,PCP]] \n");
				return 0;
			}

			cntx->cmd[OPTION_VLAN_TAG] = true;
			break;
		case 'e':
			if (!cntx->cmd[WRITE_CONFIG])
				return -EINVAL;
			cntx->cmd[OPTION_ENABLE_CONF] = true;
			if (parse_enable(optarg, cntx) != 0) {
				fprintf(stderr, "Invalid enable option. Valid option are [mac, vlan, discard ][ disable] \n");
				return 0;
			}
			break;
		default:
			fprintf(stderr, "error: unknown param\n");
			return -EINVAL;
		}
	}

	if (2 * nopt != argc -1)
		return -EINVAL;

	if (cntx->cmd[WRITE_CONFIG] && !nopt)
		return -EINVAL;

	return 0;
}

int parse_cmd_to_cntx(int argc, char **argv, struct qepctl_cntx *cntx)
{

	char *end;
	int ret = 0;

	if (!argv[1]){
		fprintf(stderr, "No Arguments \n");
		return -EINVAL;
	}

	if ((strcasecmp(argv[1], "config")== 0) ||
		(strcasecmp(argv[1], "conf" ) == 0)) {
		cntx->cmd[WRITE_CONFIG] = true;
		argc--;
		argv++;
	}
	else if (strcasecmp(argv[1], "show") == 0) {
		cntx->cmd[SHOW_CONFIG] = true;
		argc--;
		argv++;
	}

	else if (strcasecmp(argv[1], "read") == 0) {

		cntx->cmd[READ_REG] = true;
		argc--;
		argv++;
		if (argc > 1 ) {
			cntx->offset = strtoul(argv[1], &end, 0);
			if (strlen(end)) {
				fprintf(stderr, "Invalid Number Argument\n");
				return -EINVAL;
			}
			argc--;
			argv++;
		} else {
			fprintf(stderr, "Invalid Number of Arguments\n");
			return -EINVAL;
		}
	}
	else if (strcasecmp(argv[1], "write") == 0) {
		cntx->cmd[WRITE_REG] = true;
		argc--;
		argv++;
		if (argc > 1) {
			cntx->offset = strtoul(argv[1], &end, 0);
			if (strlen(end)) {
				fprintf(stderr, "Invalid Number Argument\n");
				return -EINVAL;
			}
			argc--;
			argv++;
		} else {
			fprintf(stderr, "Invalid Number of Arguments\n");
			return -EINVAL;
		}

		if (argc > 1) {
			cntx->val = strtoul(argv[1], &end, 0);
			if (strlen(end)) {
				fprintf(stderr, "Invalid Number Argument\n");
				return -EINVAL;
			}
			argc--;
			argv++;
		} else {
			fprintf(stderr, "Invalid Number of Arguments\n");
			return -EINVAL;
		}
	}
	else{
		fprintf(stderr, "Invalid  Input CMD \n");
		usage();
		return -EINVAL;
	}

	ret = parse_options(argc, argv, cntx);
	if (ret){
		fprintf(stderr, "Invalid Options \n");
		return -EINVAL;
	}

	return 0;
}

int main_core(int argc, char **argv, struct qepctl_cntx *cntx)
{
	int ret = 0;
	void *base = NULL;
	uint32_t dev_id = PCIE_DEV_ID_MGMT_PF;

	ret = parse_cmd_to_cntx(argc, argv, cntx);
	if (ret != 0) {
		fprintf(stderr, "Invalid CMDline input \n");
		return ret;
	}

	if (!cntx->cmd[OPTION_USER_BDF]) {
		ret = bdf_from_devid(dev_id, cntx->bdf, MAX_BDF_LEN);
		if (ret == -EINVAL)
			return ret;
	}

	base = mmap_pci_bar(cntx->bdf, cntx->bar_num);
	if (base == NULL) {
		fprintf(stderr, "mamap failed \n");
		return -EINVAL;
	}

	if (cntx->cmd[SHOW_CONFIG]) {
		ret = qepsec_config_show(base, cntx);
		goto base_unmap;
	}

	if (cntx->cmd[READ_REG]){
		cntx->val = readl(base + cntx->offset);
		fprintf(stdout, " bdf:%s bar:%d Value at addr:0x%x is 0x%x \n",
				cntx->bdf, cntx->bar_num, cntx->offset, cntx->val);
		goto base_unmap;
	}

	if (cntx->cmd[WRITE_REG]){
		writel(cntx->val, base + cntx->offset);
		fprintf(stdout, " bdf:%s bar:%d Value at addr:0x%x is 0x%x \n",
				cntx->bdf, cntx->bar_num, cntx->offset, ret);
		goto base_unmap;
	}
	if (cntx->cmd[WRITE_CONFIG]) {
		ret = qepsec_config(base, cntx);
		goto base_unmap;
	}

base_unmap:
	munmap(base, DEFAULT_USER_BAR_LEN);
	if (ret != 0)
		fprintf(stderr, "CMD failed \n");

	return ret;
}

int main(int argc, char **argv)
{
	struct qepctl_cntx cntx;

	if (argc <= 1) {
		fprintf(stderr, "Invalid CMD \n");
		usage();
		return -EINVAL;
	}

	if ( (strcasecmp(argv[1], "help") == 0) ||
	     (strcasecmp(argv[1], "--help") == 0) ||
	      (strcasecmp(argv[1], "-h") == 0)) {
		usage();
		return 0;
	}

	memset(&cntx, 0, sizeof(struct qepctl_cntx));

	return main_core(argc, argv, &cntx);
}
