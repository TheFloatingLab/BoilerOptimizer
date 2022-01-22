/* BoilerControl. See https://www.thefloatinglab.world/en/boilers.html for detailed description.
 * Copyright 2022 Frans Veldman (www.thefloatinglab.world)
 * Released under GNU General Public License v3.0
 * 
 * Voltage measuring
 * =================
 * To get the reference voltage within the windows of the ADC's, we need to bring VCCref down to max 5V and we need to bring up GNDref to allow for "negative" GNDref deviations
 * 
 *                      ADC1            ADC0
 *             _______   |    _______   |   _______
 *    VCCref --|_____|---|----|_____|---|---|_____|---- GNDref 
 *                R1            R2            R3
 *
 * The approximate ratio for the resistor divider is 10:3:1.5, like 10K, 3K3, 1K5, which loads the reference voltage supply with less than 1mA.
 * If we assume a maximum voltage of 15V, then this would give 4.86V on the input of ADC1 and 1,52V on the input of ADC0.
 * If we assume a scenario like a firm load on the bus (inverter?) so the voltage sags to 12V, with an additional loss of 0.5V on the negative lead of the power supply,
 * it would give 3.39V on ADC1 and 0.71V on ADC0 (remember in this scenario the ADC's get biased by the 0.5V "elevated" ADC GND).
 * 
 * The math:
 * ((ADC1-ADC0)*AREF/1024+1) * (R1+R2+R3)/R2
 * 
 * If we use option "NoRef" we just measure ADC1 against GND. The formula would then be:
 * ADC1*AREF/(1024+1) * (R1+R2+R3)/(R2+R3)
 * 
 * For a 24V system, there should be a 15K resistor in series with R1.
 * 
 */

// Debug
//#define SIMTEMP               300
//#define USEDEFAULT

// Electrical properties
#define ADCREF                5000  // Voltage of the 5V power supply in millivolts
#define R1                    10000 // Ohms
#define R2                    3300  // Ohms
#define R3                    1500  // Ohms
// Inputs
#define BUTTON                2
#define GNDREF                A0
#define VCCREF                A1
#define TEMP                  6
#define ENGINE                9
// Outputs
#define HEATER                13
#define AUX                   12
#define BACKLIGHT             5
// Interfaces
// #define LCDADDR               0x3F  // I2C address, uncomment for I2C display
#define LCD_RS                10
#define LCD_EN                11
#define LCD_D4                3
#define LCD_D5                4
#define LCD_D6                7
#define LCD_D7                8

// Constants
#define VERSION               "1.0"
#define EPROMCHK              0x7874
#define SHORTPRESS            25    // Minimum time in milliseconds to register a button press as valid instead of just a noise spike.
#define LONGPRESS             650   // Time in milliseconds to register a key press as a long press.
#define DBLPRESS              500   // max milliseconds between keypresses to consider them double clicks.
#define MENUTIMEOUT           10    // Timeout in seconds to return to default operation from within the menu.
#define VSAMPLES              100   // Voltage samples to average to cancel out noise.

// Defaults after factory reset (can be changed in the menu afterwards)
#define BRIGHTNESS            4     // LCD backlight brightness
#define DISPTIMEOUT           0     // LCD backlight timeout in seconds
#define VREQ                  141   // Required voltage*10 to switch on the heater automatically.
#define VFLOAT                135   // Float voltage*10
#define VOFF                  125   // Voltage*10 at which a pause has to be initiated
#define PAUSE                 20    // Duration of the pause in minutes
#define TMIN                  40    // Antifreeze protection should be initiated below this temperature (Celsius*10)
#define TTARGET               400   // Target temperature (Celsius*10) for daily usage
#define TSANITY               600   // Temperature (Celsius*10) needed to clear the "sanity alert"
#define TMAX                  800   // Maximum allowed temperature (Celsius*10)
#define TNOSAN                250   // Don't display a sanity alert below this temperature (Celsius*10)
#define TSANDAYS              7     // Interval in days between reaching the "sanity temperature" to not raise the "sanity alert"
#define PUMPDELTA             250   // Difference in input and boiler temperature in Celsius*10
#define PUMPAFTERRUN          60    // Time in seconds to continue running the pump after PUMPDELTA is no longer met

// Constants that are not configurable in the menu.
#define RETARD                60    // Seconds to wait after last heater state change before switching it on again.
#define VINTERVAL             500   // Display interval of voltage
#define TRENDDELTA            3     // Temperature (Celsius*10) difference required to change the trend arrow

struct rom_t {
    uint16_t epromchk; uint8_t brightness; uint8_t disptimeout; uint16_t vreq; uint16_t vfloat; uint16_t voff; int16_t pause; boolean fahrenheit; int16_t tmin; int16_t ttarget; 
    int16_t tsanity; int16_t tmax; int16_t tsandays; int16_t tnosan; int16_t pumpdelta; boolean noref; uint16_t pumpafterrun; uint8_t pvdirect;
} rom = {EPROMCHK,BRIGHTNESS,DISPTIMEOUT,VREQ,VFLOAT,VOFF,PAUSE,false,TMIN,TTARGET,TSANITY,TMAX,TSANDAYS,TNOSAN, PUMPDELTA, false, PUMPAFTERRUN, 0};

struct sensors_t {
   uint16_t epromchk; 
   uint8_t temphwa[2][8];
} sensors = {EPROMCHK,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

struct cal_t {
    uint16_t epromchk;
    uint16_t calibration;
} cal = {EPROMCHK,1000};


struct mma_t {
    uint16_t samples;
    uint16_t avg;
    uint32_t sum;
};

struct tempdata_t {
    int16_t temperature;
    int16_t failures;
    int16_t fallback;
    mma_t mmaTemp;
};

tempdata_t tempdata[] = {{0,0,0,1000},{0,0,0,-1000}};


struct button_t {
    const uint8_t button;
    uint32_t pressed;          // Button press duration
    boolean longpressed;
} button = {BUTTON,0,false};


// menu code, 8 bits
#define MENU_8        B00001000
#define MENU_16       B00010000
#define MENU_SIGNED   B10000000
#define MENU_FLOAT    B00000011
#define MENU_BOOLEAN  B01000000
#define MENU_TEMP     B00000100


struct menu_t {
    const char txt[17];
    const char sfx[13];
    const uint8_t code;
    const int16_t min;
    const uint16_t max;
    const uint16_t step;
    const void* varptr;
};

const menu_t PROGMEM menu[]= {
    {"LCD Brightness",  "",         MENU_8,                           1,    8,    1,    &rom.brightness},
    {"LCD TimeOut",     " seconds", MENU_8,                           0,    10,   1,    &rom.disptimeout},
    {"Start Voltage",   "V",        MENU_16+1,                        138,  148,  1,    &rom.vreq},
    {"Float Voltage",   "V",        MENU_16+1,                        125,  140,  1,    &rom.vfloat},
    {"Abort Voltage",   "V",        MENU_16+1,                        120,  135,  1,    &rom.voff},
    {"Pause duration",  " minutes", MENU_16+MENU_SIGNED,              5,    45,   5,    &rom.pause},
    {"Fahrenheit",      "",         MENU_BOOLEAN,                     0,    1,    1,    &rom.fahrenheit},
    {"Min Temperature", "",         MENU_16+MENU_SIGNED+MENU_TEMP+1,  0,    100,  10,   &rom.tmin},
    {"Target Temp",     "",         MENU_16+MENU_SIGNED+MENU_TEMP+1,  340,  520,  20,   &rom.ttarget},
    {"Max Temperature", "",         MENU_16+MENU_SIGNED+MENU_TEMP+1,  600,  900,  50,   &rom.tmax},
    {"Sanity Temp",     "",         MENU_16+MENU_SIGNED+MENU_TEMP+1,  560,  680,  20,   &rom.tsanity},
    {"Sanity interval", " days",    MENU_16+MENU_SIGNED,              2,    14,   1,    &rom.tsandays},
    {"No Sanity below", "",         MENU_16+MENU_SIGNED+MENU_TEMP+1,  200,  300,  10,   &rom.tnosan},
    {"V Calibration",   "",         MENU_16,                          920,  1080, 1,    &cal.calibration},
    {"No Voltage Ref",  "",         MENU_BOOLEAN,                     0,    1,    1,    &rom.noref},
    {"Pump delta",      "",         MENU_16+MENU_SIGNED+MENU_TEMP+1,  0,    100,  5,    &rom.pumpdelta},
    {"Pump afterrun",   " seconds", MENU_16,                          0,    300,  30,   &rom.pumpafterrun},
    {"PV direct feed",  "",         MENU_8,                           0,    2,    1,    &rom.pvdirect},
    {NULL,0,0,0,0,NULL}
};

const char mode0[] PROGMEM = "Automatic";
const char mode1[] PROGMEM = "Pause";
const char mode2[] PROGMEM = "WarmUp";
const char mode3[] PROGMEM = "Sanitize";
const char mode4[] PROGMEM = "Thermostat";
const char mode5[] PROGMEM = "AntiFreeze";
const char mode6[] PROGMEM = "Heater On";
const char mode7[] PROGMEM = "Pump on";
const char mode8[] PROGMEM = "Off";
const char mode9[] PROGMEM = "Setup";

const char* const modes[] = {mode0,mode1,mode2,mode3,mode4,mode5,mode6,mode7,mode8,mode9,NULL};

uint8_t mode=0;
uint8_t menusel=0;
boolean menuedit=false;
int16_t lastTemperature=0;

boolean lcdon=true;
boolean romdirty=false;
boolean paused=true;
boolean restartTmrReq=true;

uint32_t disptimer=0;
uint32_t menuseltimer=0;
uint32_t pauseTmr=0;
uint32_t restartTmr=0;
uint32_t pumpTimer=0;

uint16_t rawVoltages[5];

#ifdef LCDADDR
// Libraries
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(LCDADDR,16,2);   // set the LCD address to LCDADDR for a 16 chars and 2 line display
#else
#include <LiquidCrystal.h>
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
#endif

#include <OneWire.h>
OneWire ds(TEMP);            // Connect 1-wire devices


void setup() {
    pinMode(BUTTON,INPUT_PULLUP);
    pinMode(ENGINE,INPUT_PULLUP);
    pinMode(HEATER,OUTPUT);
    pinMode(AUX,OUTPUT);
    pinMode(BACKLIGHT,OUTPUT);

    digitalWrite(HEATER,LOW);
    digitalWrite(AUX,LOW);

    pinMode(BACKLIGHT,OUTPUT);
    analogWrite(BACKLIGHT,(32*BRIGHTNESS)-1);

    bitSet (DIDR0, ADC2D);  // disable digital buffer on A2
    bitSet (DIDR0, ADC3D);  // disable digital buffer on A3

    initLCD();

    boolean resetpressed=false;
    uint32_t tmr=millis();
    while(millis()-tmr<3000) {
        if(!digitalRead(BUTTON))
            resetpressed=true;
    }

#ifndef SIMTEMP
    // Read the sensor addresses
    eeprom_read_block((void*)&sensors, (void*)128, sizeof(sensors));

    // If the reset button is pressed and we already have sensor addresses, offer the user to reset the sensor addresses as well
    if(resetpressed && sensors.epromchk==EPROMCHK) {
        lcd.clear();
        lcd.print(F("Hold button to"));
        lcd.setCursor(0,1);
        lcd.print(F("reset sensors"));
        delay(5000);
    }
    lcd.clear();
    
    // Button still pressed or sensor addresses never initialized?
    if(!digitalRead(BUTTON) || sensors.epromchk!=EPROMCHK) {
        lcd.print(F("Sensors reset"));
        delay(2000);
        sensors.epromchk=EPROMCHK;
        searchSensors();
        if(!sensors.temphwa[0][0]) {
            lcd.print(F("no sensor found"));
            while(true);
        }
        eeprom_write_block((void*)&sensors, (void*)128, sizeof(sensors));
    }
#endif

    // Setup sensors to measure temperatures
    ds.reset();
    ds.write(0xCC); // Skip command, next command is for all devices
    ds.write(0x44); // Temperature conversion

    // Read the calibration values and reset them if factory new
    eeprom_read_block((void*)&cal, (void*)196, sizeof(2));
    if(cal.epromchk!=EPROMCHK) {
        cal.epromchk=EPROMCHK;
cal.calibration=1020;        
        eeprom_write_block((void*)&cal, (void*)196, sizeof(cal));
    }
    eeprom_read_block((void*)&cal, (void*)196, sizeof(cal));

    // Read the configuration settings from ROM
    eeprom_read_block((void*)&rom, (void*)0, sizeof(2));

    // Reset key pressed or ROM not yet initialized?
    if(resetpressed || rom.epromchk!=EPROMCHK) {
        // Perform factory reset
        rom.epromchk=EPROMCHK;
        lcd.print(F("Hold button to"));
        lcd.setCursor(0,1);
        lcd.print(F("Factory Reset"));
        delay(5000);
        lcd.clear();
        if(!digitalRead(BUTTON))
            eeprom_write_block((void*)&rom, (void*)0, sizeof(rom));
    }

#ifndef USEDEFAULT
    eeprom_read_block((void*)&rom, (void*)0, sizeof(rom));                   // These are the configuration settings.
    analogWrite(BACKLIGHT,(32*rom.brightness)-1);
#endif

    // Prime the Voltage median array
    for (int i = 0; i < 5; i++) {
        rawVoltages[i] = readvoltage(10);
        delay(15 + i * 10); // Use a non constant delay to avoid any pulsing loads to sync with the sampling frequency
    }
    // Collect some voltage samples
    for (int i = 0; i < VSAMPLES; i++)
        getvoltage();

    uint16_t voltage=getvoltage();
    // If it looks like the voltage is connected to the 5V power source, perform a calibration.
    // The 5V power source is used by the ADC as a reference (ADCREF), so this calibration works even if the 5V is not very precise.
    if(voltage<65 && voltage>35) {
        // We don't care about the exact values of the resistors, we only care about the ratio between (R1+R2+R3) and R2.
        lcd.clear();
        lcd.print(F("Calibrating..."));
        voltage=0;
        for(uint8_t i=0; i<10; i++) {  // We need EXACTLY 10 samples, so we can compare the result directly against ADCREF which is in milliVolts.
            voltage+=readvoltage(1);
            delay(10*i);
        }
        // We now have the voltage, it should be 5000, the same as ADCREF.
        cal.calibration=(uint16_t)( ((1000L * (uint32_t)voltage) +(ADCREF/2) ) / ADCREF );
        lcd.setCursor(0,0);
        lcd.print(F("Calibration ok!"));
        delay(2000);
        if(digitalRead(BUTTON) && rom.epromchk==EPROMCHK)
            eeprom_write_block((void*)&cal, (void*)196, sizeof(cal));
        while(true) {
            lcd.setCursor(0,1);
            lcd.print("Value ");
            lcd.print(cal.calibration);
            lcd.print("=");
            lcd.print(((float)getvoltage())/10.0,1);
            lcd.print("V");
            delay(1000);
        }
    }

    // Prime the "history" of the temperature. Make sure we start with a downward trend
    delay(1500);
    lastTemperature=readtemp(0)+TRENDDELTA;
}


void loop() {
    int16_t butt=chkbutton(&button);

    // Deal with the LCD backlight
    if(butt) {
        // We had a keypress, reset LCD backlight timeout
        disptimer=seconds();
        // If the backlight was not on, turn it on now
        if(!lcdon) {
            analogWrite(BACKLIGHT,(32*rom.brightness)-1);
            lcdon=true;
            butt=0; // This keypress is "used up" to switch the LCD backlight on.
        }
    }

    if(butt) {
        menuseltimer=seconds();
        if(butt==2) {
            if(menusel) {
                if(menusel>=100) {
                    menuedit=!menuedit;
                    lcd.clear();
                    printMenu(menusel-100,menuedit,rom.fahrenheit);
#ifndef SIMTEMP
                    if(!menuedit && romdirty) {
                        eeprom_write_block((void*)&rom, (void*)0, sizeof(rom));
                        eeprom_write_block((void*)&cal, (void*)196, sizeof(cal));
                    }
#endif
                    
                } else {
                    if(modes[menusel]==NULL) {
                        menusel=99;
                        butt=1; 
                    } else {
                        paused=false;
                        mode=menusel-1;
                        menusel=0;
                        lcd.clear();
                    }
                }
            }
        } 
        if(butt==1) {
            lcd.clear();
            if(menuedit) {
                romdirty=true;
                editItem(menusel-100);
                printMenu(menusel-100,menuedit,rom.fahrenheit);
                if(!(menusel-100))
                    analogWrite(BACKLIGHT,(32*rom.brightness)-1);
            } else {
                menusel++;
                if(menusel>=100) {
                    if(getPgmVal(menu[menusel-100].txt)==NULL)
                        menusel=100;
                    printMenu(menusel-100,menuedit,rom.fahrenheit);
                } else {
                    if(modes[menusel-1]==NULL)
                        menusel=1;

                    lcd.print("> ");
                    pgmLcdPrint(modes[menusel-1]);
                    lcd.setCursor(0,1);
                    lcd.print(F("LongPress=Select"));
                }
            }
        }        
    } else {
        if(menusel==113) {
            lcd.setCursor(11,1);
            lcd.print((float)getvoltage()/10.0,1);
            lcd.write('V');
        }
        if(menusel && seconds()-menuseltimer>(MENUTIMEOUT)) {
            menusel=0;
            menuedit=false;
            romdirty=false;
            lcd.clear();
        }
        if(!menusel) {
            processmodes();
        }
    }
}


void processmodes() {
    uint16_t voltage=getvoltage();
    static int16_t LCDvoltage;
    static uint32_t vfreq;

    // Did the display backlight timeout?
    if(lcdon && rom.disptimeout && seconds()-disptimer>rom.disptimeout) {
        lcdon=false;
        analogWrite(BACKLIGHT,0);
    }

    // Change the value displayed on the LCD only twice per second
    if(!LCDvoltage || millis()-vfreq>VINTERVAL) {
        LCDvoltage=voltage;
        vfreq=millis();
    }
    
    lcd.setCursor(0,0);
    lcd.print(F("Boiler "));
    printTemperature(readtemp(0),1,rom.fahrenheit);
    if(readtemp(0)<100)
        lcd.write(' ');

    lcd.setCursor(0,1);
    pgmLcdPrint(modes[mode]);

    lcd.setCursor(11,1);
    lcd.print(((float)LCDvoltage)/10.0,1);
    lcd.print("V");

    // Update the temperature trend arrow
    lcd.setCursor(13,0);
    if(readtemp(0)-lastTemperature>=TRENDDELTA) {
        lcd.write(6);
        lastTemperature=readtemp(0);
    } else
    if(lastTemperature-readtemp(0)>=TRENDDELTA) {
        lcd.write(5);
        lastTemperature=readtemp(0);
    }
        
    lcd.setCursor(14,0);
    if(sanitycheck())
        lcd.write(4);
    else
        lcd.write(' ');
    
    lcd.setCursor(15,0);
    if(!mode && !digitalRead(ENGINE))
        lcd.write(1);
    else
    if(!mode && rom.pvdirect && digitalRead(AUX))
        lcd.write(2);
    else
    if(paused)
        lcd.write(7);
    else
    if(digitalRead(HEATER))
        lcd.write(3);
    else
        lcd.write(' ');

    // If the voltage is below the float voltage, note it, so for automatic mode we need to restart the timer
    if(voltage<rom.vfloat)
        restartTmrReq=true;

    // If Temp is higher than max, switch the boiler and pump off.
    if(readtemp(0)>rom.tmax) {
        boilerState(false);
        pumpState(LOW);
        return; // Nothing else needs to be done.
    }

    // If voltage below the lower threshold, pause whatever program is running
    if(voltage<rom.voff) {
        boilerState(false);
        pumpState(LOW);   // The pump also consumes power, so let's keep it off until the voltage goes up.
        paused=true;
        pauseTmr=0;
    }

    // If Paused, keep boiler off until the timer expires
    if(paused || mode==1) {
        // Was the mode set manually to pause? Then we actually need to start the pause timer.
        if(!paused) {
            pauseTmr=0;
            paused=true;
        }
        if(voltage>=rom.vreq) {
            // Did we already start the countdown?
            if(!pauseTmr) {
                pauseTmr=seconds();
            } else {
                // Did the timer expire?
                if(seconds()-pauseTmr>rom.pause*60L) {
                    paused=false;
                    if(mode==1)     // If the pause mode was manually selected...
                        mode=0;     // ... reset it to automatic mode
                } else {
                    // Nope, we still need to pause
                    boilerState(false);
                    // Do not return, maybe solar can still input some heat
                }
            }
        }
    }

    // We've handled all hard limits, now look at the modes

    // If mode is OFF, shut down everything, we are finished.
    if(mode==8) {
        boilerState(false);
        pumpState(LOW);
        return; // No further actions required.    
    }

    // Is mode is PUMP, keep on the pump.
    if(mode==7) {
        pumpState(HIGH);
        return;
    }

    // If we have a solar pump, is there a worthwile temperature difference between solarpanel and boiler?
    if(readtemp(1)-readtemp(0)>rom.pumpdelta) {
        pumpState(HIGH);
        boilerState(false);
        pumpTimer=seconds();
        return;       // We are warming the boiler, as long as we can do that no electricity is needed, skip the rest.
    } else {
        if(seconds()-pumpTimer>rom.pumpafterrun)
            pumpState(LOW);
    }

    if(paused) {
        boilerState(false);
        return;
    }
    
    // If program=ON, just keep the boiler on. Overtemp and pause is already dealt with above.
    if(mode==6) {
        boilerState(true);
        return;
    }

    // If program is WarmUp, set/keep the boiler on when T<ttarget, otherwise return to auto
    if(mode==2) {
        if(readtemp(0)<rom.ttarget) {
            boilerState(true);
            return;
        } else {
            mode=0;
        }
    }

    // If program is Sanitize, set the boiler on when T<tsanity, otherwise return to auto
    if(mode==3) {
        if(readtemp(0)<rom.tsanity) {
            boilerState(true);
            return;
        } else {
            mode=0;
        }
    }

    // If program is Thermostat, set boiler on when T<ttarget (or when T<<tsanity if required), otherwise switch it off.
    if(mode==4) {
        if(readtemp(0)<rom.ttarget) {
            boilerState(true);
        } else {
            boilerState(sanitycheck());
        }
        return;
    }

    // If we arrive here, we are either running in auto or antifreeze mode.

    // Boiler about to freeze? Turn it on.
    if(readtemp(0)<rom.tmin) {
        boilerState(true);
        return;
    }

    // If antifreeze mode, since we got here the water is apparently warm enough, so we are ready now.
    if(mode==5) {
        boilerState(false);
        return;
    }

    // If we make it until here, we are in automatic mode

    // Is the engine running? Then we don't have to use electricity
    if(!digitalRead(ENGINE)) {
        boilerState(false);
        return;
    }

    // Is the boiler off? Maybe we can switch it on?
    if(!digitalRead(HEATER)) {
        // If voltage too low, don't do anything
        if(voltage<rom.vreq)
            return;
            
        // The voltage is high, so check to see if we need to reset the timer
        if(restartTmrReq) {
            restartTmrReq=false;
            restartTmr=seconds();
        }

        uint32_t holdoff=1;
        boolean usePVdirect=false;
            
        // Did we already reach the target temperature, then we can be a bit more conservative with energy consumption
        if(readtemp(0)>rom.ttarget) {
            if(sanitycheck()) {
                holdoff=2;  // We are due to run a sanity cycle, so let's try to reach it with moderate conservatism
            } else {
                holdoff=5;  // Sanity cycle is not due, so let's not try too hard to get the temperature up to the sanity level
                if(rom.pvdirect==1)
                    usePVdirect=true;   // PVdirect option 1, so use "PV direct"
            }
        }

        if(rom.pvdirect>=2)
            usePVdirect=true;     // PVdirect option 2, always use "PV direct" in automatic mode

        // Did we reach the sanity temperature?
        if(readtemp(0)>rom.tsanity)
            holdoff=10;     // We just heat the water further but only if we are otherwise waisting available energy.
                
        // Now we need to see how long ago we reached (again) VREQ. Can we already start?
        if(seconds()-restartTmr>holdoff*60L)
            _boilerState(true,usePVdirect);

        return;
    }

    // If we arrive here, the boiler is on. Do we want to keep it that way?

    // If boiler is on but we get under the float voltage, switch it off
    if(voltage<rom.vfloat) {
        boilerState(false);
        return;
    }
}


void boilerState(boolean state) {
    _boilerState(state,false);
}


void _boilerState(boolean state, boolean pvdirect) {
    static uint32_t lastChange=0;
    // If we want to switch the heater on, do so only after at least some time has passed since the last shutdown.
    if(!state || millis()-lastChange>RETARD*1000L) {
        if(pvdirect) {
            digitalWrite(HEATER,LOW);
            if(digitalRead(AUX)!=state) {
                delay(20);
                lastChange=millis();
                digitalWrite(AUX,state);
            }
        } else {
            digitalWrite(AUX,LOW);
            if(digitalRead(HEATER)!=state) {
                delay(20);
                lastChange=millis();
                digitalWrite(HEATER,state);
            }        
        }
    }
}


void pumpState(boolean state) {
    if(rom.pvdirect)
        return;
    digitalWrite(AUX,state);
}


// This routine will keep track whether there is a legionella chance
boolean sanitycheck() {
    static boolean sancounting=false;
    static boolean keepalarm=false;       // Prevent the alarm switching off after timer rollover
    static uint32_t santotal=0;
    static uint32_t sancount=0x80000000;
    
    if(readtemp(0)>=rom.tsanity) {
        // The temperature in boiler is high enough to kill legionella, so clear all alarms.
        santotal=0;
        sancounting=false;
        keepalarm=false;
    } else {
        if(readtemp(0)>rom.tnosan) {
            if(!sancounting) {
                sancounting=true;
                sancount=seconds();
            }
            if(santotal+(seconds()-sancount)>((uint32_t)rom.tsandays)*60L*60L*24L) {
                keepalarm=true;
            }
        } else {
            if(sancounting) {
                sancounting=false;
                santotal+=(seconds()-sancount);
            }
        }
    }
    return keepalarm;
}
    

void printMenu(uint8_t item, boolean edit, boolean fahrenheit) {
    uint8_t code = getPgmVal(&menu[item].code);
    uint16_t value = getRomValue(code,item);

    if(!edit)
        lcd.write('>');
    pgmLcdPrint(menu[item].txt);
    lcd.setCursor(0,1);
    if(edit)
        lcd.print("> ");

    if(code & (MENU_8 | MENU_16)) {
        if(code & MENU_FLOAT)
            if(code & MENU_TEMP)
                printTemperature(value,0,fahrenheit);
            else
                lcd.print((float) ((float)value/pow(10.0,(float)(code & MENU_FLOAT))), code & MENU_FLOAT);
        else
            if(code & MENU_SIGNED)
                lcd.print((int)value);
            else
                lcd.print(value);
    }
    
    if(code & MENU_BOOLEAN)
        if(value)
            lcd.print("Yes");
        else
            lcd.print("No ");
}    


uint16_t getRomValue(uint8_t code, uint8_t item) {
    if(code & (MENU_8 | MENU_BOOLEAN)) {
        if(code & MENU_SIGNED)
            return (uint16_t)((int16_t)*((int8_t *)pgm_read_word(&menu[item].varptr)));
        else
            return (uint16_t)*((uint8_t *)pgm_read_word(&menu[item].varptr));
    }

    if(code & MENU_16)
        return (uint16_t *)*((uint16_t *)pgm_read_word(&menu[item].varptr));
}


void editItem(uint8_t item) {
    uint8_t code = getPgmVal(&menu[item].code);
    uint16_t value = getRomValue(code,item);

    value+=getPgmVal(&menu[item].step);
    value-=value%getPgmVal(&menu[item].step);
    if(value>getPgmVal(&menu[item].max) || value<getPgmVal(&menu[item].min))
        value=getPgmVal(&menu[item].min);

    if(code & (MENU_8 | MENU_BOOLEAN)) {
        if(code & MENU_SIGNED)
            *((int8_t *)pgm_read_word(&menu[item].varptr)) = (int8_t)value;
        else
            *((uint8_t *)pgm_read_word(&menu[item].varptr)) = (uint8_t)value;
    }

    if(code & MENU_16)
        *((uint16_t *)pgm_read_word(&menu[item].varptr)) = value;
}


uint16_t readvoltage(uint32_t mag) {
    uint32_t vccref;
    uint32_t gndref;
    uint32_t rdiv=R2;
    vccref=analogRead(VCCREF);
    if(rom.noref) {
        gndref=0;
        rdiv+=R3;
    } else {
        // When alternating between two analog ports, throw the first result away
        vccref=analogRead(VCCREF);
        gndref=analogRead(GNDREF);
        gndref=analogRead(GNDREF);
    }
    return (uint16_t)((((((((vccref-gndref)*(R1+R2+R3))/rdiv)*ADCREF)+(cal.calibration/2))/(uint32_t)cal.calibration)+(mag*5))/(mag*10));
}


uint16_t getvoltage() {
    static uint8_t index=0;
    static mma_t RefVoltage;

    rawVoltages[index++]=readvoltage(10);
    index%=5;

    return modMovAvg(&RefVoltage,medianp(rawVoltages),VSAMPLES);
}


int16_t readtemp(uint8_t sensor) {
    uint8_t data[9];
    static uint32_t temptimer=0;

#ifdef SIMTEMP
    return SIMTEMP;
#endif

    // No hardware address for this sensor slot?
    if(!sensors.temphwa[sensor][0])
        return tempdata[sensor].fallback; // Return a "safe" but indicative value.

    if((!tempdata[sensor].failures && millis()-temptimer<10000) || (tempdata[sensor].failures && millis()-temptimer<1000))
        return tempdata[sensor].temperature;

    temptimer=millis();

    // Read the temperatures, they should be ready by now
    for(uint8_t x=0;x<sizeof(sensors.temphwa)/sizeof(sensors.temphwa[0]);x++) {
        // Is this hardware address initialized?
        if(!sensors.temphwa[x][0])
            break;
            
        ds.reset();
        ds.select(sensors.temphwa[x]);
        // Issue Read scratchpad command
        ds.write(0xBE);
        // Receive 9 bytes
        for(byte i = 0; i < 9; i++) {
            data[i] = ds.read();
        }

        // Check what we've got. Is the data valid?
        if(OneWire::crc8(data,8)==data[8]) {
            // Yup, data is valid. Convert it and draw it through our averaging filter
            tempdata[x].temperature = modMovAvg(&tempdata[x].mmaTemp,( (data[1] << 8) + data[0] ) * 10 / 16, 10);
            tempdata[x].failures=0;
        } else {
            tempdata[x].failures++;
        }
    }

    // Setup the sensors for measuring the temperature, so we have new data ready at the next reading attempt
    ds.reset();
    ds.write(0xCC); // Skip command, next command is for all devices
    ds.write(0x44); // Temperature conversion
    
    if(tempdata[sensor].failures>10)
        // Too many failures, start returning a "safe" but unusual value.
        return tempdata[sensor].fallback;

    return tempdata[sensor].temperature;
}


void searchSensors() {
    uint8_t owaddr[8];
    uint8_t sensor=0;
  
    sensors.temphwa[0][0]=0;
    sensors.temphwa[1][0]=0;

    ds.reset();
    while(ds.search(owaddr) && sensor<2) {
        // We have found a one-wire device. Is it a temperature sensor and is the data valid?
        if(owaddr[0]==0x28 && OneWire::crc8( owaddr, 7) == owaddr[7]) {
            // Seems to be ok. So copy this hardware address
            for(uint8_t i=0;i<8;i++)
                sensors.temphwa[sensor][i]=owaddr[i];
            // Now we have it, configure the sensor
            ds.reset();
            ds.select(sensors.temphwa[sensor]);
            ds.write(0x4E);     // Write scratchpad command
            ds.write(0);        // TL data
            ds.write(0);        // TH data
            ds.write(0x7F);     // Configuration Register (resolution) 7F=12bits 5F=11bits 3F=10bits 1F=9bits
            ds.reset();         // This "reset" sequence is mandatory, it allows the DS18B20 to understand the copy scratchpad to EEPROM command
            ds.select(sensors.temphwa[sensor]);
            ds.write(0x48);     // Copy Scratchpad command
            sensor++;
            ds.reset();
        }
    }
}


int16_t getPgmVal(const int16_t *ptr) {
    return pgm_read_word(ptr);
}

uint16_t getPgmVal(const uint16_t *ptr) {
    return pgm_read_word(ptr);
}

char getPgmVal(const char *ptr) {
    return pgm_read_byte(ptr);
}

byte getPgmVal(const byte *ptr) {
    return pgm_read_byte(ptr);
}

// Prints a string from flash memory
const char* pgmLcdPrint(const char *txt) {
    char c=pgm_read_byte_near(txt);
    while(c) {
        lcd.write(c);
        txt++;
        c=pgm_read_byte_near(txt);
    }
    txt++;
    return txt;
}


void printTemperature(int16_t value,uint8_t precision, boolean fahrenheit) {
    if(fahrenheit) {
        lcd.print(((float)value*0.18)+32.0,precision);
        lcd.print("\xDF""F");
    } else {
        lcd.print((float)value/10.0,precision);
        lcd.print("\xDF""C");
    }
}


void initLCD() {
    // Special characters for LCD
    uint8_t Anchor[8]  = {0b00100,0b01010,0b00100,0b00100,0b00100,0b10101,0b01110,0b00000};
    uint8_t Sailing[8] = {0b00010,0b00110,0b01111,0b01111,0b11111,0b00010,0b11111,0b01110};
    uint8_t solar[8]   = { B11111, B11111, B01110, B00000, B10101, B10101, B10101, B00000};
    uint8_t legion[8]  = { B01010, B01010, B01010, B01000, B01010, B01000, B01111, B00000};
    uint8_t down[8]    = { B00100, B00100, B00100, B00100, B10101, B01110, B00100, B00000};
    uint8_t up[8]      = { B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00000};
    uint8_t lightning[8]={ B00011, B00110, B01100, B11110, B00110, B01100, B01000, B10000};
    uint8_t Tmr[8]     = { B00000, B01110, B10101, B10111, B10001, B01110, B00000, B00000};
    uint8_t engine[8]  = { B00000, B01111, B00100, B01111, B11111, B01111, B00000, B00000};

#ifdef LCDADDR
    lcd.begin();
#else
    lcd.begin(16, 2);
#endif
    lcd.createChar(1, Anchor);
    lcd.createChar(2, Sailing);

    delay(100);

    for(int offset=0;offset<6;offset++) {
        lcd.setCursor(offset,0);
        lcd.print(" ");
        lcd.write((uint8_t)2);
        delay(400);
    }
    delay(800);
    lcd.setCursor(6,1);
    lcd.write((uint8_t)1);
    delay(1000);

    lcd.clear();
    lcd.print(F("BoilerControl" VERSION));
    lcd.setCursor(0,1);
    lcd.print(F("by Frans Veldman"));

    lcd.createChar(1, engine);
    lcd.createChar(2, solar);
    lcd.createChar(3, lightning);
    lcd.createChar(4, legion);
    lcd.createChar(5, down);
    lcd.createChar(6, up);
    lcd.createChar(7, Tmr);
}


uint32_t seconds() {
    static uint32_t secs=0;
    static uint32_t prevmillis=0;

    uint32_t dsecs=(millis()-prevmillis)/1000L;
    if(dsecs) {
          secs+=dsecs;
          prevmillis+=dsecs*1000L;
    }
    return secs;
}


// This function handles the button functionality. Button presses are separated into long, short and double clicks presses.
int chkbutton(struct button_t* button) {
    if(digitalRead(button->button)==LOW) {  // button pressed
        if(button->pressed==0)
            button->pressed=millis();
        if(millis()-button->pressed>=LONGPRESS) {
            if(!button->longpressed) {
                button->longpressed=true;
                return 2;
            }
        }
        return 0;
    } else {                              // button released
        if(button->longpressed) {
            button->longpressed=false;
            button->pressed=0;
        }
        if(button->pressed==0)
            return 0;
        unsigned long pressed=millis()-button->pressed;
        button->pressed=0;
        if(pressed<SHORTPRESS)
            return 0;
        if(pressed<LONGPRESS) {
            return 1;
        }
        return 0;
    }
}


// ******** Modified Moving Average filter *********

uint16_t modMovAvg(mma_t* mma, const uint16_t val, const uint16_t maxSamples) {
  if(mma->samples<maxSamples)
    mma->samples++;         // This is going to be an additional sample
  else
    mma->sum-=mma->avg;     // Max amount of samples reached, substract average to make room for new value
  mma->sum+=val;
  mma->avg=(uint16_t)((mma->sum+(mma->samples>>1))/mma->samples);
  return mma->avg;
}

// ******** Routines to find the median in an array of 5 values *********

// Trick using XOR to swap two variables
#define swap(a,b) a ^= b; b ^= a; a ^= b;
#define sort(a,b) if(a>b){ swap(a,b); }

// http://cs.engr.uky.edu/~lewis/essays/algorithms/sortnets/sort-net.html
// Median could be computed two less steps...
uint16_t median(uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e) {
  sort(a, b);
  sort(d, e);
  sort(a, c);
  sort(b, c);
  sort(a, d);
  sort(c, d);
  sort(b, e);
  sort(b, c);
  // this last one is obviously unnecessary for the median
  //sort(d,e);

  return c;
}

uint16_t medianp(uint16_t values[5]) {
  return median(values[0], values[1], values[2], values[3], values[4]);
}
