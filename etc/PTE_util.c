/** 
 * Test code for process command handling
 * Based off of adcs example process
 *
 */

#include <polysat/polysat.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <ctype.h>

#define MAX_PASS 3000

struct MulticallInfo;
struct IMUData;

static int PTE_start(int, char**, struct MulticallInfo *);
static int PTE_safe(int, char**, struct MulticallInfo *);
static int PTE_active(int, char**, struct MulticallInfo *);
static int PTE_status(int, char**, struct MulticallInfo *);
static int IMU_trigger(int, char**, struct MulticallInfo *);

// struct holding all possible function calls
// running the executable with the - flags will call that function
// running without flags will print out this struct
struct MulticallInfo {
   int (*func)(int argc, char **argv, struct MulticallInfo *);
   const char *name;
   const char *opt;
   const char *help;
} multicall[] = {
   { &PTE_start, "PTE_start", "-I", 
       "Start PTE -I" }, 
   { &PTE_safe, "PTE_safe", "-S", 
       "Put PTE in safe mode -S" }, 
	{ &PTE_active, "PTE_active", "-A", 
       "PUT PTE in active mode -A" }, 
	{ &PTE_status, "PTE_status", "-D",
	   "Get PTE status -D" },
	{ &IMU_trigger, "IMU_trigger", "-T",
	   "Mock data from IMU -T"},
   { NULL, NULL, NULL, NULL }
};

struct IMUData {
	double t[MAX_PASS];
	double x[MAX_PASS];
	double y[MAX_PASS];
	double z[MAX_PASS];
	double temp[MAX_PASS];
};

static int IMU_trigger(int argc, char **argv, struct MulticallInfo * self) 
{
	
	int cmd = 5;
   
   struct RespData {
		double resp_altered;
		int listen;
	};
	
	struct {
		uint8_t cmd;
		struct RespData {
			double resp_altered;
			int listen;
		} resp_data;
    } __attribute__((packed)) resp;

   struct {
      uint8_t cmd;
	  double send_data;
   } __attribute__((packed)) send;

   send.cmd = cmd;
   send.send_data = 1;
   const char *ip = "224.0.0.1";
   int len, opt;
   
   while ((opt = getopt(argc, argv, "h:")) != -1) {
      switch(opt) {
         case 'h':
            ip = optarg;
            break;
      }
   }
   

   // send packet
   if ((len = socket_send_packet_and_read_response(ip, "test1", &send, 
    sizeof(send), &resp, sizeof(resp), 2000)) <= 0) {
      return len;
   } // error if less than 0

   if (resp.cmd != CMD_STATUS_RESPONSE) {
	printf("response code incorrect, got 0x%02X expected 0x%02x\n", resp.cmd, CMD_STATUS_RESPONSE);
	return 5;
   }

   printf("Sent: %lf Received: %lf Should be: %lf\n", send.send_data, resp.resp_data.resp_altered, send.send_data + 1 - 2*!resp.resp_data.listen);
   
   return 0;
}

static int PTE_start(int argc, char **argv, struct MulticallInfo * self) 
{
	
	int cmd = 2;
   
	struct {
	uint8_t cmd;
	struct PTEStatus {
		int listen;
		int mode;
	} flags;
    } __attribute__((packed)) resp;

   struct {
      uint8_t cmd;
   } __attribute__((packed)) send;

   send.cmd = cmd;
   const char *ip = "224.0.0.1";
   int len, opt;
   
   while ((opt = getopt(argc, argv, "h:")) != -1) {
      switch(opt) {
         case 'h':
            ip = optarg;
            break;
      }
   }
   

   // send packet
   if ((len = socket_send_packet_and_read_response(ip, "test1", &send, 
    sizeof(send), &resp, sizeof(resp), 2000)) <= 0) {
      return len;
   } // error if less than 0

   if (resp.cmd != CMD_STATUS_RESPONSE) {
	printf("response code incorrect, got 0x%02X expected 0x%02x\n", resp.cmd, CMD_STATUS_RESPONSE);
	return 5;
   }

   printf("Listening status: %d\n", resp.flags.listen);
   if (resp.flags.mode)
	printf("PTE mode: ACTIVE_MODE\n");
   else
	printf("PTE mode: SAFE_MODE\n");

   return 0;
}

/* 
 * @param argc number of command line arguments
 * @param argv char array of command line arguments
 * @return 0 on succes, failure otherwise
 */
static int PTE_safe(int argc, char **argv, struct MulticallInfo * self) 
{
	
	int cmd = 3;
   
	struct {
	uint8_t cmd;
	struct PTEStatus {
		int listen;
		int mode;
	} flags;
    } __attribute__((packed)) resp;

   struct {
      uint8_t cmd;
   } __attribute__((packed)) send;

   send.cmd = cmd;
   const char *ip = "224.0.0.1";
   int len, opt;
   
   while ((opt = getopt(argc, argv, "h:")) != -1) {
      switch(opt) {
         case 'h':
            ip = optarg;
            break;
      }
   }
   

   // send packet
   if ((len = socket_send_packet_and_read_response(ip, "test1", &send, 
    sizeof(send), &resp, sizeof(resp), 2000)) <= 0) {
      return len;
   } // error if less than 0

   if (resp.cmd != CMD_STATUS_RESPONSE) {
	printf("response code incorrect, got 0x%02X expected 0x%02x\n", resp.cmd, CMD_STATUS_RESPONSE);
	return 5;
   }

   printf("Listening status: %d\n", resp.flags.listen);
   if (resp.flags.mode)
	printf("PTE mode: ACTIVE_MODE\n");
   else
	printf("PTE mode: SAFE_MODE\n");

   return 0;
}

static int PTE_active(int argc, char **argv, struct MulticallInfo * self) 
{
	
	int cmd = 4;
   
	struct {
	uint8_t cmd;
	struct PTEStatus {
		int listen;
		int mode;
	} flags;
    } __attribute__((packed)) resp;

   struct {
      uint8_t cmd;
   } __attribute__((packed)) send;

   send.cmd = cmd;
   const char *ip = "224.0.0.1";
   int len, opt;
   
   while ((opt = getopt(argc, argv, "h:")) != -1) {
      switch(opt) {
         case 'h':
            ip = optarg;
            break;
      }
   }
   

   // send packet
   if ((len = socket_send_packet_and_read_response(ip, "test1", &send, 
    sizeof(send), &resp, sizeof(resp), 2000)) <= 0) {
      return len;
   } // error if less than 0

   if (resp.cmd != CMD_STATUS_RESPONSE) {
	printf("response code incorrect, got 0x%02X expected 0x%02x\n", resp.cmd, CMD_STATUS_RESPONSE);
	return 5;
   }

   printf("Listening status: %d\n", resp.flags.listen);
   if (resp.flags.mode)
	printf("PTE mode: ACTIVE_MODE\n");
   else
	printf("PTE mode: SAFE_MODE\n");

   return 0;
}

static int PTE_status(int argc, char **argv, struct MulticallInfo * self) 
{
	
	int cmd = 1;
   
	struct {
	uint8_t cmd;
	struct PTEStatus {
		int pass;
		double threshold;
		long double delta_V;
		long double error;
		long double estimation;
		int listen;
		int mode;
	} flags;
    } __attribute__((packed)) resp;

   struct {
      uint8_t cmd;
   } __attribute__((packed)) send;

   send.cmd = cmd;
   const char *ip = "224.0.0.1";
   int len, opt;
   
   while ((opt = getopt(argc, argv, "h:")) != -1) {
      switch(opt) {
         case 'h':
            ip = optarg;
            break;
      }
   }
   

   // send packet
   if ((len = socket_send_packet_and_read_response(ip, "test1", &send, 
    sizeof(send), &resp, sizeof(resp), 2000)) <= 0) {
      return len;
   } // error if less than 0

   if (resp.cmd != CMD_STATUS_RESPONSE) {
	printf("response code incorrect, got 0x%02X expected 0x%02x\n", resp.cmd, CMD_STATUS_RESPONSE);
	return 5;
   }

   printf("Listening status: %d\n", resp.flags.listen);
   if (resp.flags.mode)
	printf("PTE mode: ACTIVE_MODE\n");
   else
	printf("PTE mode: SAFE_MODE\n");
	printf("Pass #: %d\n", resp.flags.pass);
	printf("Threshold: %lf\n", resp.flags.threshold);
	printf("DeltaV: %Lf\n", resp.flags.delta_V);
	printf("PTE: %Lf\n", resp.flags.estimation);

   return 0;
}

// prints out available commands for this util
static int print_usage(const char *name)
{
   struct MulticallInfo *curr;

   printf("PTE-util multicall binary, use the following names instead:\n");

   for (curr = multicall; curr->func; curr++) {
      printf("   %-16s %s\n", curr->name, curr->help);
   }

   return 0;
}

int main(int argc, char **argv) 
{   
   struct MulticallInfo *curr;
   char *exec_name;

   exec_name = rindex(argv[0], '/');
   if (!exec_name) {
      exec_name = argv[0];
   }
   else {
      exec_name++;
   }

   for (curr = multicall; curr->func; curr++) {
      if (!strcmp(curr->name, exec_name)) {
         return curr->func(argc, argv, curr);
      }
   }

   if (argc > 1) {
      for (curr = multicall; curr->func; curr++) {
         if (!strcmp(curr->opt, argv[1])) {
            return curr->func(argc - 1, argv + 1, curr);
         }
      }
   }
   else {
      return print_usage(argv[0]);
   }

   return 0;
}
