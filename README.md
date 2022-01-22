This is the open source version of the BoilerOptimizer as described at https://www.thefloatinglab.world/en/boilers.html

This BoilerOptimizer is intended for controlling the Boiler on sailboats, features are:

- Uses the excess energy of the solar panels, once the batteries are fully charged, to heat the electric boiler
- Can optionally be used to feed the boiler directly from the solar panels, so the heating element will be "modulated" with the available solar energy.
- It will instantly switch off the heating element when a heavy consumer switches on (anchor winch etc).
- Energy saving of 66% by heating up to 40&deg;C instead of 60&deg;C.
- Fast warmup mode to just "shower temperature" so you can take a shower for just 1/3th of the energy.
- Legionella prevention and warning, by periodically heating the water to a bacteria killing temperature or warning when it has been too long ago that this temperature could be reached.
- Thermostatic control when connected to shore power
- Can optionally control a pump based on a differential temperature, so you can build your own "solar panel" consisting of nothing more than just a hose on deck and a small electric pump.
- Guards the boiler against freezing
- Automatic energy saving when it detects that the engine is running and coolant can be used to heat up the boiler.
- Reliable voltage measurement with optional voltage reference inputs
- LCD display with boiler temperature, trend, voltage and heater status
- Fully configurable by an on screen menu
- Switchable between degrees Celsius and Fahrenheit

See for details and background https://www.thefloatinglab.world/en/boilers.html

## Notes about the Boiler Optimizer Schematic

### Explanation of some less obvious parts

- R1, R2 and R3 are used to bring the reference voltage into the window of the ADC's. R1 together with C1 also services as a *low pass filter* to filter out induction peaks. The function of R3 is to be able to measure GNDref voltages wich are slightly more *negative* than GND.

### Components

- You can use an Aduino UNO (or clone) or an Arduino Pro Mini (5V, 32Kb). Some other variants probably will work too but you might have to change the PIN definitons in the software.
- If you use an Arduino UNO, you can probably use its internal voltage regulator and ommit the 78L05. If you use an Arduino Pro Mini, you will definitely have to use the 78L05 like shown in the schematic.
- For use on 24V systems, you will have to change R1 into 27K and R4 into 18K, and you will need a 78L05 voltage regulator even for the Arduino UNO. The current version of the software doesn't support 24V yet.
- The relation between R1, R2 and R3 is important. If you change any of these values, you *MUST* change their definitions in the software too! You don't need high precision resistors; the software will be able to calibrate around small deviations.
- Q1 and Q2 can be any NPN transistor, D1 and D2 are generic diodes.
- The LCD is a generic 16x2 display (make sure it is a 5V version), you can use an I2C variant as well but then you have to adapt the schematic and uncomment the relevant definition in the software.
- If you want to connect the solar panels directly to the boiler, you *MUST* use a solid state relay and wire everything exactly like shown in "Installation". A normal mechanical relay can not break a high voltage DC current; *it will arc and melt*! The mechanical relay in the minus is ok, the solid state relay in the plus will break the arc.

### Commissioning

- The LCD might stay blank or black until the contrast pot has been tuned. It is best to test and adjust this before executing the software on the Arduino for the first time.
- Connect the temperature sensor(s). Perform a factory reset (will be done automatically if it is the first time the software runs) and the software will start pairing the sensor(s). If you use more than one sensor, you have to find out which one is for the boiler and which is for the solar panel. Some hot water will be handy with testing. ;-)
- Connect the voltage reference inputs to the power supply (5V!) of the Arduino and reboot. This will automatically start the calibration. Once the calibration is completed, power down and connect the voltage reference inputs to 12V.
