%{
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>

	extern FILE *yyin;
	extern void yyerror(const char *);
	int yydebug = 1;

	//#define MAX_RX_BUCKETS 128

	struct tx_stn {
		const char *name;
		const char *tbl_name;
	};

	struct tx_stn tx_stns[] = {
		{ .name = "TX_STN_CPU_0" , .tbl_name = "cpu_0" },
		{ .name = "TX_STN_CPU_1" , .tbl_name = "cpu_1" },
		{ .name = "TX_STN_CPU_2" , .tbl_name = "cpu_2" },
		{ .name = "TX_STN_CPU_3" , .tbl_name = "cpu_3" },
		{ .name = "TX_STN_CPU_4" , .tbl_name = "cpu_4" },
		{ .name = "TX_STN_CPU_5" , .tbl_name = "cpu_5" },
		{ .name = "TX_STN_CPU_6" , .tbl_name = "cpu_6" },
		{ .name = "TX_STN_CPU_7" , .tbl_name = "cpu_7" },

		{ .name = "TX_STN_XGS_0" , .tbl_name = "xgs_0" },
		{ .name = "TX_STN_XGS_1" , .tbl_name = "xgs_1" },

		{ .name = "TX_STN_GMAC" , .tbl_name = "gmac" },

		{ .name = "TX_STN_DMA" , .tbl_name = "dma" },

		{ .name = "TX_STN_SEC" , .tbl_name = "sec" },

		{ 0 }
	};

#define MAX_TX_STNS ((int)(sizeof(tx_stns)/sizeof(struct tx_stn)) - 1)

	struct rx_stn {
		const char *name;
		int base_bucket;
		int num_buckets;
		int rx_entries;
	};

	struct rx_stn rx_stns[] = {
		{ "RX_STN_CPU_0", 0, 8, 256},
		{ "RX_STN_CPU_1", 8, 8, 256},
		{ "RX_STN_CPU_2", 16, 8, 256},
		{ "RX_STN_CPU_3", 24, 8, 256},
		{ "RX_STN_CPU_4", 32, 8, 256},
		{ "RX_STN_CPU_5", 40, 8, 256},
		{ "RX_STN_CPU_6", 48, 8, 256},
		{ "RX_STN_CPU_7", 56, 8, 256},

		{ "RX_STN_XGS_0_TX", 64, 16, 256},
		{ "RX_STN_XGS_1_TX", 80, 16, 256},

		{ "RX_STN_GMAC", 96, 8, 256},
   
		{ "RX_STN_DMA", 104, 8, 256 },

		{ "RX_STN_XGS_0_FR", 112, 2, 256},
		{ "RX_STN_XGS_1_FR", 114, 2, 256},
   
		{ "RX_STN_SEC", 120, 8, 256 },
   
		{0, 0, 0, 0}
	};

#define MAX_RX_STNS ((int)(sizeof(rx_stns)/sizeof(struct rx_stn)) - 1)

	struct rx_bucket {
		const char *name;   
		int size;
		int tx_credits[MAX_TX_STNS];
		int rx_stn;
		const char *rx_stn_name;
	};
 
	static struct rx_bucket rx_buckets[] = {
		{.name = "cpu_0_0" , .rx_stn_name = "RX_STN_CPU_0" },
		{.name = "cpu_0_1" , .rx_stn_name = "RX_STN_CPU_0" },
		{.name = "cpu_0_2" , .rx_stn_name = "RX_STN_CPU_0" },
		{.name = "cpu_0_3" , .rx_stn_name = "RX_STN_CPU_0" },
		{.name = "cpu_0_4" , .rx_stn_name = "RX_STN_CPU_0" },
		{.name = "cpu_0_5" , .rx_stn_name = "RX_STN_CPU_0" },
		{.name = "cpu_0_6" , .rx_stn_name = "RX_STN_CPU_0" },
		{.name = "cpu_0_7" , .rx_stn_name = "RX_STN_CPU_0" },
   
		{.name = "cpu_1_0" , .rx_stn_name = "RX_STN_CPU_1" },
		{.name = "cpu_1_1" , .rx_stn_name = "RX_STN_CPU_1" },
		{.name = "cpu_1_2" , .rx_stn_name = "RX_STN_CPU_1" },
		{.name = "cpu_1_3" , .rx_stn_name = "RX_STN_CPU_1" },
		{.name = "cpu_1_4" , .rx_stn_name = "RX_STN_CPU_1" },
		{.name = "cpu_1_5" , .rx_stn_name = "RX_STN_CPU_1" },
		{.name = "cpu_1_6" , .rx_stn_name = "RX_STN_CPU_1" },
		{.name = "cpu_1_7" , .rx_stn_name = "RX_STN_CPU_1" },
   
		{.name = "cpu_2_0" , .rx_stn_name = "RX_STN_CPU_2" },
		{.name = "cpu_2_1" , .rx_stn_name = "RX_STN_CPU_2" },
		{.name = "cpu_2_2" , .rx_stn_name = "RX_STN_CPU_2" },
		{.name = "cpu_2_3" , .rx_stn_name = "RX_STN_CPU_2" },
		{.name = "cpu_2_4" , .rx_stn_name = "RX_STN_CPU_2" },
		{.name = "cpu_2_5" , .rx_stn_name = "RX_STN_CPU_2" },
		{.name = "cpu_2_6" , .rx_stn_name = "RX_STN_CPU_2" },
		{.name = "cpu_2_7" , .rx_stn_name = "RX_STN_CPU_2" },
   
		{.name = "cpu_3_0" , .rx_stn_name = "RX_STN_CPU_3" },
		{.name = "cpu_3_1" , .rx_stn_name = "RX_STN_CPU_3" },
		{.name = "cpu_3_2" , .rx_stn_name = "RX_STN_CPU_3" },
		{.name = "cpu_3_3" , .rx_stn_name = "RX_STN_CPU_3" },
		{.name = "cpu_3_4" , .rx_stn_name = "RX_STN_CPU_3" },
		{.name = "cpu_3_5" , .rx_stn_name = "RX_STN_CPU_3" },
		{.name = "cpu_3_6" , .rx_stn_name = "RX_STN_CPU_3" },
		{.name = "cpu_3_7" , .rx_stn_name = "RX_STN_CPU_3" },
   
		{.name = "cpu_4_0" , .rx_stn_name = "RX_STN_CPU_4" },
		{.name = "cpu_4_1" , .rx_stn_name = "RX_STN_CPU_4" },
		{.name = "cpu_4_2" , .rx_stn_name = "RX_STN_CPU_4" },
		{.name = "cpu_4_3" , .rx_stn_name = "RX_STN_CPU_4" },
		{.name = "cpu_4_4" , .rx_stn_name = "RX_STN_CPU_4" },
		{.name = "cpu_4_5" , .rx_stn_name = "RX_STN_CPU_4" },
		{.name = "cpu_4_6" , .rx_stn_name = "RX_STN_CPU_4" },
		{.name = "cpu_4_7" , .rx_stn_name = "RX_STN_CPU_4" },
   
		{.name = "cpu_5_0" , .rx_stn_name = "RX_STN_CPU_5" },
		{.name = "cpu_5_1" , .rx_stn_name = "RX_STN_CPU_5" },
		{.name = "cpu_5_2" , .rx_stn_name = "RX_STN_CPU_5" },
		{.name = "cpu_5_3" , .rx_stn_name = "RX_STN_CPU_5" },
		{.name = "cpu_5_4" , .rx_stn_name = "RX_STN_CPU_5" },
		{.name = "cpu_5_5" , .rx_stn_name = "RX_STN_CPU_5" },
		{.name = "cpu_5_6" , .rx_stn_name = "RX_STN_CPU_5" },
		{.name = "cpu_5_7" , .rx_stn_name = "RX_STN_CPU_5" },
   
		{.name = "cpu_6_0" , .rx_stn_name = "RX_STN_CPU_6" },
		{.name = "cpu_6_1" , .rx_stn_name = "RX_STN_CPU_6" },
		{.name = "cpu_6_2" , .rx_stn_name = "RX_STN_CPU_6" },
		{.name = "cpu_6_3" , .rx_stn_name = "RX_STN_CPU_6" },
		{.name = "cpu_6_4" , .rx_stn_name = "RX_STN_CPU_6" },
		{.name = "cpu_6_5" , .rx_stn_name = "RX_STN_CPU_6" },
		{.name = "cpu_6_6" , .rx_stn_name = "RX_STN_CPU_6" },
		{.name = "cpu_6_7" , .rx_stn_name = "RX_STN_CPU_6" },
   
		{.name = "cpu_7_0" , .rx_stn_name = "RX_STN_CPU_7" },
		{.name = "cpu_7_1" , .rx_stn_name = "RX_STN_CPU_7" },
		{.name = "cpu_7_2" , .rx_stn_name = "RX_STN_CPU_7" },
		{.name = "cpu_7_3" , .rx_stn_name = "RX_STN_CPU_7" },
		{.name = "cpu_7_4" , .rx_stn_name = "RX_STN_CPU_7" },
		{.name = "cpu_7_5" , .rx_stn_name = "RX_STN_CPU_7" },
		{.name = "cpu_7_6" , .rx_stn_name = "RX_STN_CPU_7" },
		{.name = "cpu_7_7" , .rx_stn_name = "RX_STN_CPU_7" },

		{.name = "xgs_0_tx_0" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_1" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_2" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_3" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_4" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_5" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_6" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_7" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_8" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_9" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_10" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_11" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_12" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_13" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_14" , .rx_stn_name = "RX_STN_XGS_0_TX" },
		{.name = "xgs_0_tx_15" , .rx_stn_name = "RX_STN_XGS_0_TX" },

		{.name = "xgs_1_tx_0" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_1" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_2" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_3" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_4" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_5" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_6" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_7" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_8" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_9" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_10" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_11" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_12" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_13" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_14" , .rx_stn_name = "RX_STN_XGS_1_TX" },
		{.name = "xgs_1_tx_15" , .rx_stn_name = "RX_STN_XGS_1_TX" },

		{.name = "gmac_rsvd_0" , .rx_stn_name = "RX_STN_GMAC" },
		{.name = "gmac_rfr_0" , .rx_stn_name = "RX_STN_GMAC" },
		{.name = "gmac_tx_0" , .rx_stn_name = "RX_STN_GMAC" },
		{.name = "gmac_tx_1" , .rx_stn_name = "RX_STN_GMAC" },
		{.name = "gmac_tx_2" , .rx_stn_name = "RX_STN_GMAC" },
		{.name = "gmac_tx_3" , .rx_stn_name = "RX_STN_GMAC" },
		{.name = "gmac_rsvd_1" , .rx_stn_name = "RX_STN_GMAC" },
		{.name = "gmac_rfr_1" , .rx_stn_name = "RX_STN_GMAC" },

		{.name = "dma_chan_0", .rx_stn_name = "RX_STN_DMA" },
		{.name = "dma_chan_1", .rx_stn_name = "RX_STN_DMA" },
		{.name = "dma_chan_2", .rx_stn_name = "RX_STN_DMA" },
		{.name = "dma_chan_3", .rx_stn_name = "RX_STN_DMA" },
   
		{.name = "rsvd_0", .rx_stn_name = "RX_STN_RSVD" },
		{.name = "rsvd_1", .rx_stn_name = "RX_STN_RSVD" },
		{.name = "rsvd_2", .rx_stn_name = "RX_STN_RSVD" },
		{.name = "rsvd_3", .rx_stn_name = "RX_STN_RSVD" },

		{.name = "xgs_0_rsvd", .rx_stn_name = "RX_STN_XGS_0_FR" },
		{.name = "xgs_0_rfr", .rx_stn_name = "RX_STN_XGS_0_FR" },

		{.name = "xgs_1_rsvd", .rx_stn_name = "RX_STN_XGS_1_FR" },
		{.name = "xgs_1_rfr", .rx_stn_name = "RX_STN_XGS_1_FR" },

		{.name = "rsvd_4", .rx_stn_name = "RX_STN_RSVD" },
		{.name = "rsvd_5", .rx_stn_name = "RX_STN_RSVD" },
		{.name = "rsvd_6", .rx_stn_name = "RX_STN_RSVD" },
		{.name = "rsvd_7", .rx_stn_name = "RX_STN_RSVD" },

		{.name = "sec_pipe_0", .rx_stn_name = "RX_STN_SEC" },
		{.name = "sec_pipe_1", .rx_stn_name = "RX_STN_SEC" },
		{.name = "sec_pipe_2", .rx_stn_name = "RX_STN_SEC" },
		{.name = "sec_pipe_3", .rx_stn_name = "RX_STN_SEC" },
		{.name = "sec_rsa", .rx_stn_name = "RX_STN_SEC" },
		{.name = "sec_rsvd_5", .rx_stn_name = "RX_STN_SEC" },
		{.name = "sec_rsvd_6", .rx_stn_name = "RX_STN_SEC" },
		{.name = "sec_rsvd_7", .rx_stn_name = "RX_STN_SEC" },

		{.name = 0 }
	};

#define MAX_RX_BUCKETS ((int)(sizeof(rx_buckets)/sizeof(struct rx_bucket)) - 1)

	static void add_bucket(const char *name);
	static void modify_bucket_size(int size);
	static void modify_bucket_tx_credit(const char *str, int credits);

#define fatal_error(fmt, args...) {fprintf(stderr, fmt, ##args); exit(-1);}

	%}

%token TOK_BUCKET TOK_SIZE TOK_WORD TOK_INTEGER TOK_QUOTE TOK_OPEN_BRACE TOK_CLOSE_BRACE TOK_SEMICOLON 

%%

def: 
| def bucket_def 
;

bucket_def: 
TOK_BUCKET quotedname bucket_content 
{ //printf("grammar: bucket_def start: $1=%s, $2=%s \n", $1, $2);
	add_bucket((const char *)$2); } 
;

bucket_content: 
| TOK_OPEN_BRACE bucket_stmts TOK_CLOSE_BRACE	       
;
bucket_stmts: 
| bucket_stmts bucket_stmt TOK_SEMICOLON
;

bucket_stmt: 
| TOK_SIZE TOK_INTEGER 
{ //printf("grammar: $2=%d\n", $2); 
	modify_bucket_size($2);}
					    
| quotedname TOK_INTEGER
{ //printf("grammar: $1=%s, $2=%d\n", $1, $2); 
	modify_bucket_tx_credit((const char *)$1, $2);}
;

quotedname: TOK_QUOTE TOK_WORD TOK_QUOTE { $$=$2; } ;

%%

static const char *input = "msgring_shared.cfg";  
static const char *output = "../common/msgring_shared.c";  
static int debug = 0;

static int lookup_rx_stn(const char *name)
{
	int i=0;

	if (!name) return MAX_RX_STNS;

	for(i=0;rx_stns[i].name;i++) 
		if (strcasecmp(rx_stns[i].name, name)==0) break;

	return i;
}

static int lookup_rx_bucket(const char *name)
{
	int i=0;

	if (!name) return MAX_RX_BUCKETS;

	for(i=0;rx_buckets[i].name;i++) 
		if (strcasecmp(rx_buckets[i].name, name)==0) break;

	return i;
}

static void init_rx_bucket(int i)
{
	int j=0;

	rx_buckets[i].size = 0;
	for(j=0;j<MAX_TX_STNS;j++)
		rx_buckets[i].tx_credits[j] = 0;

	rx_buckets[i].rx_stn = lookup_rx_stn(rx_buckets[i].rx_stn_name);
}

static void copy_rx_bucket(struct rx_bucket *dest, struct rx_bucket *src)
{
	int i=0;
  
	dest->size = src->size;
	for (i=0;i<MAX_TX_STNS;i++)
		dest->tx_credits[i] = src->tx_credits[i];
}

static void init_defaults(void)
{
	int i=0;

	for(i=0;i<MAX_RX_BUCKETS;i++) 
		init_rx_bucket(i);
}

static void add_bucket(const char *name)
{
	int id = 0;

	id = lookup_rx_bucket(name);

	if (id == MAX_RX_BUCKETS) {
		fatal_error("Unrecognised bucket name %s\n", name);
	}

	copy_rx_bucket(&rx_buckets[id], &rx_buckets[MAX_RX_BUCKETS]);

	init_rx_bucket(MAX_RX_BUCKETS);
}

static void modify_bucket_size(int size)
{
	rx_buckets[MAX_RX_BUCKETS].size = size;
	//printf("bucket configured with size %d\n", size);
}

static void modify_bucket_tx_credit(const char *name, int credits)
{
	int i=0;

	for(i=0;tx_stns[i].name!=NULL;i++) 
		if (strcasecmp(tx_stns[i].name, name)==0) break;
	if (tx_stns[i].name) {
		rx_buckets[MAX_RX_BUCKETS].tx_credits[i] = credits;
		//printf("Tx Station %s configured with %d tx credits\n", name, credits);
	}
	else {
		fatal_error("Unknown tx station %s!\n", name);
	}
}

static void dump_config(void)
{
	int i=0, j=0;

	for(i=0;i<MAX_RX_BUCKETS;i++) {
		if (rx_stns[rx_buckets[i].rx_stn].name && rx_buckets[i].size) {
			printf("bucket %3d: size = %3d, rx_stn = %s, tx_credits: ", i, rx_buckets[i].size, 
			       rx_stns[rx_buckets[i].rx_stn].name);
			for(j=0;j<MAX_TX_STNS;j++) 
				if (rx_buckets[i].tx_credits[j]) 
					printf("<%s, %d> ", tx_stns[j].name, rx_buckets[i].tx_credits[j]);    
			printf("\n");
		}
	}
}

static void check_config(void)
{
	int i=0, j=0, k=0;
	int total=0;

	for(i=0;rx_stns[i].name;i++) {
		if (debug) printf("Checking \"%s\"...\n", rx_stns[i].name);
		total = 0;
		for(j=rx_stns[i].base_bucket;j<rx_stns[i].base_bucket+rx_stns[i].num_buckets;j++) {
      
			if (rx_buckets[j].size) {

				{
					int num_ones = 0;
					for(k=0;k<8;k++) if (rx_buckets[j].size & (1<<k)) num_ones++;
					if (num_ones > 1) 
						fatal_error("Bucket(%d)'s size(%d) is not a power of 2\n", j, rx_buckets[j].size);
				}

				if (total % rx_buckets[j].size) 
					fatal_error("Bucket(%d)'s base address(%d) is not aligned to it's size(%d)\n", 
						    j, total, rx_buckets[j].size);

				if (rx_buckets[j].size < 4) 
					fatal_error("Bucket(%d) size(%d) is bad, has to be 0 or >=4\n", j, rx_buckets[j].size);
			}

			total += rx_buckets[j].size;
		}
		//printf("\trx_entries=%d, configured size = %d\n", rx_stns[i].rx_entries, total);
		if (total > rx_stns[i].rx_entries) {
			fatal_error("Total configured bucket size for %s exceeds available rx entries\n", 
				    rx_stns[i].name);
		}
		for(j=rx_stns[i].base_bucket;j<rx_stns[i].base_bucket+rx_stns[i].num_buckets;j++) {
			total = 0;
			for(k=0;k<MAX_TX_STNS;k++) 
				total += rx_buckets[j].tx_credits[k];
			//printf("\tbucket %d: size = %d, total configured credits = %d\n", 
			//j, rx_buckets[j].size, total);
			if (total > rx_buckets[j].size) {
				fatal_error("Total tx credits to bucket_%d exceed the bucket size\n", j);
			}
		}  
	}
	if (debug) printf("...done\n");
} 

static void gen_tables(void)
{
	FILE *fp = fopen(output, "w");
	char line[1024];
	int i=0, j=0;

	if (!output) {
		fatal_error("Unable to write to %s\n", output);
	}

	sprintf(line, "/**********************************************************\n");
	fwrite(line, strlen(line), 1, fp);
	sprintf(line, " * -----------------DO NOT EDIT THIS FILE------------------\n");
	fwrite(line, strlen(line), 1, fp);
	sprintf(line, " * This file has been autogenerated by the build process\n");
	fwrite(line, strlen(line), 1, fp);
	sprintf(line, " * from \"%s\" \n", input);
	fwrite(line, strlen(line), 1, fp);
	sprintf(line, " **********************************************************/\n\n");
	fwrite(line, strlen(line), 1, fp);

	sprintf(line, "#include <linux/module.h>\n");
	fwrite(line, strlen(line), 1, fp);
	sprintf(line, "#include <asm/netlogic/msgring.h>\n\n");
	fwrite(line, strlen(line), 1, fp);

	/* Generate Bucket Sizes data structure */
	sprintf(line, "struct bucket_size shared_bucket_sizes = {\n\t{");
	fwrite(line, strlen(line), 1, fp);

	for (i=0;i<MAX_RX_BUCKETS;i++) {
		if (i && (i%8)==0) sprintf(line, "\n\t\t%d, ", rx_buckets[i].size);
		else sprintf(line, "%3d, ", rx_buckets[i].size);
		fwrite(line, strlen(line), 1, fp);
	}
  
	sprintf(line, "\n\t}\n};\nEXPORT_SYMBOL(shared_bucket_sizes);\n\n");
	fwrite(line, strlen(line), 1, fp);

	/* Generate Credit tables */
	for(i=0; tx_stns[i].name; i++) {
		sprintf(line, "struct stn_cc shared_cc_table_%s = {{", tx_stns[i].tbl_name);
		fwrite(line, strlen(line), 1, fp);

		for (j=0;j<MAX_RX_BUCKETS;j++) {
			if ((j % 8)==0) sprintf(line, "\n\t\t{%d", rx_buckets[j].tx_credits[i]);
			else sprintf(line, ", %d ", rx_buckets[j].tx_credits[i]);
			fwrite(line, strlen(line), 1, fp);
			if ((j % 8)==7) {sprintf(line, "},"); fwrite(line, strlen(line), 1, fp);}
		}    
		sprintf(line, "\n\t}};\nEXPORT_SYMBOL(shared_cc_table_%s);\n\n",
			tx_stns[i].tbl_name);
		fwrite(line, strlen(line), 1, fp);
	}
}

static void usage(const char *progname)
{
	fprintf(stderr, "Usage: %s -i <input file> -o <output file>\n", progname);
	exit(-1);
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int ch=0;
  
	while ( (ch=getopt(argc, argv, "hdi:o:")) != -1) {
		switch(ch) {
		case 'i': input = strdup(optarg); break;
		case 'o': output = strdup(optarg); break;
		case 'd': debug = 1; break;
			break;
		case 'h':
		default:
			usage(argv[0]);
			break;
		}
	}
	printf("Input file = \"%s\", Output file = \"%s\" \n", input, output);

	yyin = fopen(input, "r");

	if (yyin == NULL) {
		fatal_error("bad input file\n");
	}
  
	//printf("MAX_RX_STNS = %d, MAX_TX_STNS = %d, MAX_RX_BUCKETS = %d\n", 
	// MAX_RX_STNS, MAX_TX_STNS, MAX_RX_BUCKETS);

	init_defaults();
  
	ret = yyparse();

	if (ret) {
		fprintf(stderr, "Unable to Parse %s\n", input);
		return -1;
	}
	printf("Finished parsing \"%s\"\n", input);

	if (debug) dump_config();
  
	check_config();

	gen_tables();

	return 0;
}


