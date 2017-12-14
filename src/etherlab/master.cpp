/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2009-2010  Moehwald GmbH B. Benner
 *                     2011  IgH Andreas Stewering-Bone
 *                     2012  Florian Pose <fp@igh-essen.com>
 *
 *  This file is part of the IgH EtherCAT master
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT master. If not, see <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdm/rtdm.h>
#include <native/task.h>
#include <native/sem.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>
#include <pthread.h>

#include "iostream"

#include "ecrt.h"

RT_TASK my_task;

static int run = 1;

using namespace std;

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static uint8_t *domain1_pd = NULL;

static ec_slave_config_t *sc_dig_out_01 = NULL;
static ec_slave_config_t *sc_imu_01 = NULL;
static ec_slave_config_t *sc_lan9252_01 = NULL;

/****************************************************************************/

// process data

//#define BusCoupler01_Pos    0, 0
#define DigOutSlave01_Pos   0, 1


#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL2004 0x00000002, 0x07d43052


// offsets for PDO entries
static unsigned int off_dig_out0 = 0;

/********************************************************
 *GQY_IMU
 */
#define IMU_Pos             0, 0
#define GQY_IMU 0xE0000005, 0x26483052
// offsets for PDO entries
static unsigned int off_imu_gx;
static unsigned int off_imu_gy;
static unsigned int off_imu_gz;
static unsigned int off_imu_ax;
static unsigned int off_imu_ay;
static unsigned int off_imu_az;
static unsigned int off_imu_counter;
static unsigned int off_imu_led;
struct gqy_imu_data_struct
{
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    uint32_t counter;
    uint16_t led;
};
static struct gqy_imu_data_struct gqy_imu_data;

/********************************************************
 *lan9252
 */
#define WMLAN9252_IO_POS 0, 2
#define WMLAN9252_IO 0xE0000002, 0x92521000
// offsets for PDO entries
static unsigned int off_analog_data;
static unsigned int off_keys;
static unsigned int off_leds;
struct wmlan9252_io_data_struct
{
  uint16_t analog_data;
  uint8_t key0_1;
  uint8_t led0_7;
};
static struct wmlan9252_io_data_struct wmlan9252_io_data;

// process data
const static ec_pdo_entry_reg_t domain1_regs[] =
{
    {DigOutSlave01_Pos, Beckhoff_EL2004, 0x7000, 0x01, &off_dig_out0, NULL},
#if 1
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x7000, 0x01, &off_leds, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x6000, 0x01, &off_keys, NULL},
    {WMLAN9252_IO_POS,  WMLAN9252_IO, 0x6020, 0x01, &off_analog_data, NULL},
#endif

#if 1
    {IMU_Pos,  GQY_IMU, 0x6000, 0x01, &off_imu_gx, NULL},
    {IMU_Pos,  GQY_IMU, 0x6000, 0x02, &off_imu_gy, NULL},
    {IMU_Pos,  GQY_IMU, 0x6000, 0x03, &off_imu_gz, NULL},
    {IMU_Pos,  GQY_IMU, 0x6000, 0x04, &off_imu_ax, NULL},
    {IMU_Pos,  GQY_IMU, 0x6000, 0x05, &off_imu_ay, NULL},
    {IMU_Pos,  GQY_IMU, 0x6000, 0x06, &off_imu_az, NULL},
    {IMU_Pos,  GQY_IMU, 0x6000, 0x07, &off_imu_counter, NULL},
    {IMU_Pos,  GQY_IMU, 0x7011, 0x01, &off_imu_led, NULL},
#endif
    {}
};

/****************************************************************************/

/* Slave 1, "EL2004"
 * Vendor ID:       0x00000002
 * Product code:    0x07d43052
 * Revision number: 0x00100000
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
   {0x7000, 0x01, 1}, /* Output */
   {0x7010, 0x01, 1}, /* Output */
   {0x7020, 0x01, 1}, /* Output */
   {0x7030, 0x01, 1}, /* Output */
};

ec_pdo_info_t slave_1_pdos[] = {
   {0x1600, 1, slave_1_pdo_entries + 0}, /* Channel 1 */
   {0x1601, 1, slave_1_pdo_entries + 1}, /* Channel 2 */
   {0x1602, 1, slave_1_pdo_entries + 2}, /* Channel 3 */
   {0x1603, 1, slave_1_pdo_entries + 3}, /* Channel 4 */
};

ec_sync_info_t slave_1_syncs[] = {
   {0, EC_DIR_OUTPUT, 4, slave_1_pdos + 0, EC_WD_ENABLE},
   {0xff}
};

/*****************************************************************************/
//wmlan9252_io
//TxPdo
ec_pdo_entry_info_t wmlan9252_io_txpdo_entries[] = {
    {0x6000, 0x01, 8}, /* key0_2 */
    {0, 0, 8}, //reserve
    {0x6020, 0x01, 16}, /* analog_Input */
};

ec_pdo_info_t wmlan9252_io_txpdos[] = {
    {0x1A02, 1, wmlan9252_io_txpdo_entries + 2}, /* TxPdo Channel 2 */
    {0x1A00, 2, wmlan9252_io_txpdo_entries + 0}, /* TxPdo Channel 1 */
};

//RxPdo
ec_pdo_entry_info_t wmlan9252_io_rxpdo_entries[] = {
    {0x7000, 0x01, 8}, /* led0_8 */
    {0, 0, 8}, //reserve
};

ec_pdo_info_t wmlan9252_io_rxpdos[] = {
    {0x1601, 2, wmlan9252_io_rxpdo_entries + 0}, /* RxPdo Channel 1 */
};

ec_sync_info_t wmlan9252_io_syncs[] = {
    {2, EC_DIR_OUTPUT, 1, wmlan9252_io_rxpdos, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  2, wmlan9252_io_txpdos, EC_WD_ENABLE},
    {0xff}
};

/*****************************************************************************/
//GQY_IMU
//TxPdo
ec_pdo_entry_info_t gqy_imu_txpdo_entries[] = {
    {0x6000, 0x01, 32}, /* gx */
    {0x6000, 0x02, 32}, /* gy */
    {0x6000, 0x03, 32}, /* gz */
    {0x6000, 0x04, 32}, /* ax */
    {0x6000, 0x05, 32}, /* ay */
    {0x6000, 0x06, 32}, /* az */
    {0x6000, 0x07, 32}, /* counter */
};

ec_pdo_info_t gqy_imu_txpdos[] = {
    {0x1a00, 7, gqy_imu_txpdo_entries + 0}, /* TxPdo Channel 1 */
};

//RxPdo
ec_pdo_entry_info_t gqy_imu_rxpdo_entries[] = {
    {0x7011, 0x01, 16}, /* led0_8 */
};

ec_pdo_info_t gqy_imu_rxpdos[] = {
    {0x1601, 1, gqy_imu_rxpdo_entries + 0}, /* RxPdo Channel 1 */
};

ec_sync_info_t gqy_imu_io_syncs[] = {
    {2, EC_DIR_OUTPUT, 1, gqy_imu_rxpdos, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, gqy_imu_txpdos, EC_WD_ENABLE},
    {0xff}
};

/*****************************************************************************
 * Realtime task
 ****************************************************************************/

void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        rt_printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain1_state.wc_state) {
        rt_printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void rt_check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
        rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up) {
        rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/

void my_task_proc(void *arg)
{
    int cycle_counter = 0;
    unsigned int blink = 0;

    rt_task_set_periodic(NULL, TM_NOW, 1000000); // ns

    while (run) {
        rt_task_wait_period(NULL);

        cycle_counter++;

        // receive EtherCAT frames
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        rt_check_domain_state();

        if (!(cycle_counter % 1000)) {
            rt_check_master_state();
        }

        if (!(cycle_counter % 200)) {
            blink = !blink;
        }
/////////////////
// lan9252
#if 0
        {
            static int counter_txrx = 0;
            if((++counter_txrx) >= 100)
            {
                counter_txrx = 0;
                #if 1
                    // read process data
                    wmlan9252_io_data.analog_data = EC_READ_S16(domain1_pd + off_analog_data);
                    wmlan9252_io_data.key0_1 =  EC_READ_U8(domain1_pd + off_keys);
                    // write process data
                    EC_WRITE_U8(domain1_pd + off_leds, ++wmlan9252_io_data.led0_7);
                #endif
                 printf("send leds value: 0x%-4x receive ADC value: %-6d receive keys: 0x%-4x   \n\n",
                        wmlan9252_io_data.led0_7,
                        wmlan9252_io_data.analog_data,
                        wmlan9252_io_data.key0_1);
            }
        }
#endif

/////////////////////////////
/// IMU
/////////////////////////////
#if 1
        {
            static int counter_txrx = 0;
            if((++counter_txrx) >= 1000)
            {
                counter_txrx = 0;
                #if 1
                    // read process data
                    gqy_imu_data.gx = EC_READ_FLOAT(domain1_pd + off_imu_gx);
                    gqy_imu_data.gy = EC_READ_FLOAT(domain1_pd + off_imu_gy);
                    gqy_imu_data.gz = EC_READ_FLOAT(domain1_pd + off_imu_gz);
                    gqy_imu_data.ax = EC_READ_FLOAT(domain1_pd + off_imu_ax);
                    gqy_imu_data.ay = EC_READ_FLOAT(domain1_pd + off_imu_ay);
                    gqy_imu_data.az = EC_READ_FLOAT(domain1_pd + off_imu_az);
                    gqy_imu_data.counter = EC_READ_U32(domain1_pd + off_imu_counter);
                    // write process data
                    EC_WRITE_U16(domain1_pd + off_imu_led, 0xaa55);
                #endif
//                 printf("ax:%f     ay:%f        imu counter:%d      \n\n",
//                        gqy_imu_data.ax,
//                        gqy_imu_data.ay,
//                        gqy_imu_data.counter);
                std::cout << "gx:" << gqy_imu_data.gx << endl;
                std::cout << "gy:" << gqy_imu_data.gy << endl;
                std::cout << "gz:" << gqy_imu_data.gz << endl;
                std::cout << "ax:" << gqy_imu_data.ax << endl;
                std::cout << "ay:" << gqy_imu_data.ay << endl;
                std::cout << "az:" << gqy_imu_data.az << endl;
                std::cout << "counter:" << gqy_imu_data.counter << endl;
                std::cout << endl;
            }
        }
#endif

        EC_WRITE_U8(domain1_pd + off_dig_out0, blink ? 0x0 : 0x0F);

        // send process data
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }
}

/****************************************************************************
 * Signal handler
 ***************************************************************************/

void signal_handler(int sig)
{
    run = 0;
}

/****************************************************************************
 * Main function
 ***************************************************************************/

int main(int argc, char *argv[])
{
    ec_slave_config_t *sc;
    int ret;

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    printf("Requesting master...\n");
    master = ecrt_request_master(0);
    if (!master)
    {
        std::cout << "ecrt_request_master error" << std::endl;
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        std::cout << "ecrt_master_create_domain error" << std::endl;
        return -1;
    }

    printf("Creating slave configurations...\n");

    // Create configuration for bus coupler
//    sc = ecrt_master_slave_config(master, BusCoupler01_Pos, Beckhoff_EK1100);
//    if (!sc) {
//        return -1;
//    }
///////////////////////////
/// GQY IMU
///////////////////////////
    sc_imu_01 = ecrt_master_slave_config(master, IMU_Pos, GQY_IMU);
    if(!sc_imu_01)
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_imu_01, EC_END, gqy_imu_io_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
/////////////////////////////
/// lan9252
/////////////////////////////
#if 1
    sc_lan9252_01 = ecrt_master_slave_config(master, WMLAN9252_IO_POS, WMLAN9252_IO);
    if (!sc_lan9252_01)
    {
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_lan9252_01, EC_END, wmlan9252_io_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
#endif

    sc_dig_out_01 =
        ecrt_master_slave_config(master, DigOutSlave01_Pos, Beckhoff_EL2004);
    if (!sc_dig_out_01) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_dig_out_01, EC_END, slave_1_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }



    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return -1;
    }

    ret = rt_task_create(&my_task, "my_task", 0, 80, T_FPU);
    if (ret < 0) {
        fprintf(stderr, "Failed to create task: %s\n", strerror(-ret));
        return -1;
    }

    printf("Starting my_task...\n");
    ret = rt_task_start(&my_task, &my_task_proc, NULL);
    if (ret < 0) {
        fprintf(stderr, "Failed to start task: %s\n", strerror(-ret));
        return -1;
    }

    while (run) {
        sched_yield();
    }

    printf("Deleting realtime task...\n");
    rt_task_delete(&my_task);

    printf("End of Program\n");
    ecrt_release_master(master);

    return 0;
}

/***************************************************************************/
