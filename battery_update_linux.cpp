/*!
 * @file battery_update.cpp
 *
 * Battery status update for the Adafruit LC709203F Battery Monitor
 *
 * g++ -Wall battery_update.cpp adafruit_lc709203f.cpp -l pigpio -o battery_update
 *
 * BSD license (see license.txt)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include <argp.h>
#include <chrono>
#include <thread>
#include "adafruit_lc709203f_linux.h"

// #define DEBUG_PRINT

int execute_command(char *command)
{
  int status;

  pid_t pid = fork();

  if (pid == -1)
  {
    perror("Error forking process");
    return 1;
  }
  else if (pid > 0)
  { // Parent process
    waitpid(pid, &status, 0);
  }
  else
  { // Child process
    execlp("/bin/sh", "sh", "-c", command, NULL);
    perror("Error executing command");
    exit(1);
  }

  if (WIFEXITED(status))
  {
    printf("External program returned code: %d\n", WEXITSTATUS(status));
  }
  else if (WIFSIGNALED(status))
  {
    // printf("External program received signal: %d\n", WTERMSIG(status));
  }
  return 0;
}

int load_module()
{
  char command[255];
  const char *module_file_name = "/home/pi/bin/m4-integrated-battery/integrated_battery.ko";

  sprintf(command, "/sbin/rmmod %s", module_file_name);
  execute_command(command);

  sprintf(command, "/sbin/insmod %s", module_file_name);
  execute_command(command);

  return 0;
}

int update_battery_info(bool nobattery, bool cvs, bool verbose)
{
  adafruit_lc709203f_linux *battery_monitor = new adafruit_lc709203f_linux();

  if (!battery_monitor || !battery_monitor->initialize())
  {
#ifdef DEBUG_PRINT
    printf("--->Err: Cannot iniatialize battery monitor\n");
#endif
    return -1;
  }

  float voltage;
  float charge_percent;
  int charging_status = 0;
  int capacity = 5000;
  int charge_full_design = 5000;
  int charge_full = charge_full_design;
  int charge_now;
  int time_to_empty_avg = 6000;
  int voltage_now;

  charge_percent = battery_monitor->cellPercent();
  voltage = battery_monitor->cellVoltage();
  if (voltage < 1.0)
  {
    voltage = 2.0; // this allows show battery empty on error when reading voltage.
    charge_percent = 0;
  }
  charge_now = (int)((float)charge_full * ((float)charge_percent / 100.0));
  voltage_now = (int)(voltage * 1000.0);

  if (!nobattery)
  {

    FILE *fptr = fopen("/dev/integrated_battery", "w");

    if (fptr)
    {
      fprintf(fptr, "capacity0 = %d\n", capacity);
      fprintf(fptr, "charge_full_design0 = %d\n", charge_full_design);
      fprintf(fptr, "charge_full0 = %d\n", charge_full);
      fprintf(fptr, "charge_now0 = %d\n", charge_now);
      fprintf(fptr, "time_to_empty_avg0 = %d\n", time_to_empty_avg);
      fprintf(fptr, "voltage_now0 = %d\n", voltage_now);
      fprintf(fptr, "charging = %d\n", charging_status);
      fclose(fptr);
#ifdef DEBUG_PRINT
      printf("--->Battery status updated, voltage=%6.3f, charge=%5.1f\n", voltage, charge_percent);
#endif
    }
  }

  if (verbose)
  {
    if (cvs)
    {
      printf("%lu, %6.3f, %5.1f\n", (unsigned long)time(NULL), voltage, charge_percent);
    }
    else
    {
      printf("voltage=%6.3f, charge=%5.1f%%\n", voltage, charge_percent);
    }
    fflush(stdout);
  }

  delete battery_monitor;

  return 0;
}

struct arguments
{
  bool nomodule;
  bool nobattery;
  bool cvs;
  bool verbose;
};

static int parse_opt(int key, char *arg, struct argp_state *state)
{
  struct arguments *arguments = (struct arguments *)state->input;
  switch (key)
  {
  case 'm':
    arguments->nomodule = true;
    break;
  case 'b':
    arguments->nobattery = true;
    break;
  case 'c':
    arguments->cvs = true;
    break;
  case 'v':
    arguments->verbose = true;
    break;
  }
  return 0;
}

int main(int argc, char **argv)
{
  struct arguments arguments = {false, false, false, false};

  struct argp_option options[] =
      {
          {"nomodule", 'm', 0, 0, "Do not upload module"},
          {"nobattery", 'b', 0, 0, "Do not update battery info"},
          {"cvs", 'c', 0, 0, "Output as cvs"},
          {"verbose", 'v', 0, 0, "Verbose"},
          {0}};

  struct argp argp = {options, parse_opt};

  error_t status = argp_parse(&argp, argc, argv, 0, 0, &arguments);

  if (status)
  {
    return status;
  }

  if (!arguments.nomodule)
  {
    load_module();
  }

  while (true)
  {
    update_battery_info(arguments.nobattery, arguments.cvs, arguments.verbose);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }

  return 0;
}
