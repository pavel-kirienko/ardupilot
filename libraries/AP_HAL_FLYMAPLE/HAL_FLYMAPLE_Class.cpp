/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Flymaple port by Mike McCauley
 */

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include "HAL_FLYMAPLE_Class.h"
#include "AP_HAL_FLYMAPLE_Private.h"

using namespace AP_HAL_FLYMAPLE_NS;
class HardwareSerial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;

static FLYMAPLEUARTDriver uartADriver(&Serial1); // "COM1"
static FLYMAPLEUARTDriver uartBDriver(&Serial2);
static FLYMAPLEUARTDriver uartCDriver(&Serial3); // "GPS"
static FLYMAPLESemaphore  i2cSemaphore;
static FLYMAPLEI2CDriver  i2cDriver(&i2cSemaphore);
static FLYMAPLESPIDeviceManager spiDeviceManager;
static FLYMAPLEAnalogIn analogIn;
static FLYMAPLEStorage storageDriver;
static FLYMAPLEConsoleDriver consoleDriver(&uartADriver);
static FLYMAPLEGPIO gpioDriver;
static FLYMAPLERCInput rcinDriver;
static FLYMAPLERCOutput rcoutDriver;
static FLYMAPLEScheduler schedulerInstance;
static FLYMAPLEUtil utilInstance;

HAL_FLYMAPLE::HAL_FLYMAPLE() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &i2cDriver,
	&spiDeviceManager,
	&analogIn,
	&storageDriver,
        &consoleDriver,
	&gpioDriver,
	&rcinDriver,
	&rcoutDriver,
        &schedulerInstance,
	&utilInstance
	)
{}

void HAL_FLYMAPLE::init(int argc,char* const argv[]) const {
    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init(NULL);
    console->init(NULL);

    /* The AVR RCInput drivers take an AP_HAL_AVR::ISRRegistry*
     * as the init argument */
    rcin->init(NULL);
    rcout->init(NULL);
    spi->init(NULL);
    i2c->begin();
    i2c->setTimeout(100);
    analogin->init(NULL);
    storage->init(NULL); // Uses EEPROM.*, flash_stm* copied from AeroQuad_v3.2
}

const HAL_FLYMAPLE AP_HAL_FLYMAPLE;

#endif
