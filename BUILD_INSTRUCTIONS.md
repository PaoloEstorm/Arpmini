![IMG_7627](https://github.com/user-attachments/assets/04d9ebf9-eb3a-4e3d-8a49-5158be56317a)

All the components required to assemble Arpmini, including the 3D model for the case, are listed in the main repository.
Some parts will no longer be easily accessible after assembly, so make sure you have everything you need before starting, and follow the steps in the exact order.

![IMG_7628](https://github.com/user-attachments/assets/3241549b-fc8a-4c3d-aa36-a8da76474b7d)
![IMG_7629](https://github.com/user-attachments/assets/0fb7ab1c-b698-49df-aefd-05bed35ea541)

Insert the four 3.5 mm jack sockets into their corresponding holes, making sure to mount them on the correct side of the PCB; the one with the logo and SMD components.

![IMG_7633](https://github.com/user-attachments/assets/bb319975-ae6e-48fd-a4a5-4f7039b63d90)

While keeping the sockets in place, solder each exposed pin one by one, making sure the connectors stay as straight, flat, and aligned with the PCB as possible.
You can use a strip of adhesive tape to hold them steady during the process if needed.

![IMG_7637](https://github.com/user-attachments/assets/538a48ba-2530-491f-b067-3380df87bba6)

Using a flush side cutter, trim the excess length of the soldered pins as much as possible.
Be careful not to damage the PCB traces; a clean trim helps prevent short circuits in later assembly steps.

![IMG_7638](https://github.com/user-attachments/assets/12af2fa2-fd75-4b2a-a19a-13f18eb9cc27)

Take the Pro Micro board and make sure you have the male pin headers ready.
They usually come included with the board. If not, cut two 12-pin strips from a longer header.

![IMG_7640](https://github.com/user-attachments/assets/e8e18a39-80b8-4099-8b05-c478c2681255)

Most Pro Micro boards include two small pads located next to the TX pin. These pads define the board’s operating voltage:
if the pads are not bridged, the board runs at 3.3V;
if they are bridged, the board runs at 5V.

![IMG_7642](https://github.com/user-attachments/assets/097ac34c-d9d0-411b-8978-b6b37bea6ca3)

Arpmini operates at 5V, so the pads MUST be bridged using a small amount of solder.
Be careful not to damage nearby components. A fine-tip soldering iron and moderate temperature (around 280°C / 536°F) make the process easier.

![IMG_7645](https://github.com/user-attachments/assets/f51c828f-c8ba-4166-b091-382ca1d926b8)

Insert the two 12-pin male headers into the holes, making sure to place them on the side of the PCB with the "Pro Micro" label and the microcontroller.
Flip the board and solder all pins, aiming for clean joints without using too much solder, while keeping the headers pressed firmly against the board.
Using a breadboard to hold the pins in place during soldering can be very helpful!

![IMG_7643](https://github.com/user-attachments/assets/be4a3ded-f3c9-4a6e-97d1-8ecebeebbf23)

Once the headers are soldered, proceed to flash the bootloader specifically designed for Arpmini.
To connect the board to the ISP, it is recommended to use Dupont jumper cables to avoid soldering.

Alternatively, you can temporarily solder wires on the back side of the board, where the headers were just soldered, while keeping the front headers untouched.

<img width="2264" height="1798" alt="Bootloader_ProMicro_Wiring" src="https://github.com/user-attachments/assets/ba20b7d6-9e37-421d-a5eb-c4a35544c89f" />

Follow the instructions available [here](https://github.com/PaoloEstorm/Arpmini-Core) to install the Arpmini Core in the Arduino IDE.
Once installed, go to Tools -> Board -> Arpmini Core -> Arpmini.
Then, in Tools -> Programmer, select Arduino as ISP.
In Tools -> Port, select the serial port of your Arduino Uno/Nano/ISP.
Finally, click Burn Bootloader in the Tools menu.
If the connections and configuration are correct, the bootloader should be successfully written within a few seconds.

![IMG_7646](https://github.com/user-attachments/assets/03291826-cb09-4e71-a2a8-52f145933842)
![IMG_7647](https://github.com/user-attachments/assets/0e32852d-404c-4311-8915-ffcd07c3279a)

Place the Pro Micro board face down, inserting it into the holes of the main PCB as shown in the photo.

![IMG_7650](https://github.com/user-attachments/assets/33e11699-e048-4b26-836c-f3d161d3734d)

Double-check the placement before soldering!
Solder all pins, keeping the PCBs pressed firmly and aligned with each other.

Optionally, you can perform a continuity test between the front and back solder joints of each pin; if the soldering is done correctly, this step should not be necessary.

![IMG_7652](https://github.com/user-attachments/assets/8e79b010-fd77-4bfc-a82a-4693125db00d)

Using the flush side cutter, trim the excess length of the soldered pins as much as possible.

Be careful not to damage the PCB traces; a clean trim helps prevent short circuits in later assembly steps.

![IMG_7653](https://github.com/user-attachments/assets/c1932155-c225-4ad5-be54-3c565214b885)

![IMG_7657](https://github.com/user-attachments/assets/28d37a5f-f4c1-4563-8cac-c5f2120b4382)
![IMG_7659](https://github.com/user-attachments/assets/491b0f57-f502-421b-a288-3957fcad1e3c)

Place the buzzer on the top side of the main PCB, observing the correct polarity.
The hole marked (+) on the PCB must align with the (+) on the buzzer.

Double-check the placement before soldering.

![IMG_7661](https://github.com/user-attachments/assets/b448fbdd-c14b-48a4-877d-a49efdf321dc)

Using the flush side cutter, trim the excess length of the soldered pins as much as possible.

Be careful not to damage the PCB traces; a clean trim helps prevent short circuits in later assembly steps.

![IMG_7667](https://github.com/user-attachments/assets/4bda6309-fd57-4795-a7ff-9ff2edc4d1e7)

Now start assembling the screen board.
Insert the buttons into their respective slots, matching the colors marked on the PCB.
Pay special attention to the pin marked in red — this is GND and must be inserted towards the inside of the board.
For all buttons, the red-marked pins should face each other and point inward on the PCB.

To make inserting the pins easier, especially the thin ones for the LEDs, using tweezers can be helpful.

![IMG_7669](https://github.com/user-attachments/assets/32750358-4f3e-419a-a53b-eff065a1503a)
![IMG_7673](https://github.com/user-attachments/assets/432d9384-4308-4f17-a72a-e1d87877dd9c)

Before soldering, double-check that all buttons are correctly positioned.

![IMG_7674](https://github.com/user-attachments/assets/1f453460-9e71-462a-9525-08c0207658b3)
![IMG_7679](https://github.com/user-attachments/assets/ac9103a6-1a66-451a-91db-eedea5f53f19)

Using the flush side cutter, trim the excess length of the soldered pins as much as possible.

Be careful not to damage the PCB traces; a clean trim helps prevent short circuits in later assembly steps.

![IMG_7681](https://github.com/user-attachments/assets/c639f7f5-001a-402e-81d3-2fbaa15949b7)
![IMG_7683](https://github.com/user-attachments/assets/9ff5fab5-2cd6-4ad5-9677-79835a19f7b9)

Using a flush side cutter, cut out two individual male pin headers.

![IMG_7687](https://github.com/user-attachments/assets/39c085bd-aef0-4c6c-85de-9890de256990)

Place the individual headers into the two holes at the bottom of the PCB, near the yellow and green buttons.

![IMG_7688](https://github.com/user-attachments/assets/961ead4e-c635-4eb8-87cc-4ea3ac590a2a)

Then, position the screen on top of the headers and into the holes at the top of the PCB.
Screens usually come with pre-soldered pin headers; if not, cut a 7-pin strip and solder it to the screen PCB as shown in the photo above.

Be careful not to damage any PCB traces.
I recommend to keep the protective film on the screen to shield it from potential solder flux splashes.

![IMG_7690](https://github.com/user-attachments/assets/abef9f41-e62f-4ac0-be8a-ee88331872d9)

Flip the board and solder all pins, aiming for clean joints without using too much solder, while keeping the headers pressed firmly against the board.

![IMG_7694](https://github.com/user-attachments/assets/00bfa75b-9d14-4ca4-8ab5-e0a4124a3792)

Flip the PCB to the button side and solder the two support pins of the screen.

![IMG_7693](https://github.com/user-attachments/assets/1c134ed6-c941-4669-bf3f-8a0ff4c79794)

Using the flush side cutter, trim the excess length of the soldered pins as much as possible.

Be careful not to damage the PCB traces; a clean trim helps prevent short circuits in later assembly steps.

![IMG_7696](https://github.com/user-attachments/assets/a0040c43-1c60-4200-aec9-8397e92503bd)

Flip the board to the button side and trim the excess length of both the 7-pin header on the screen and the two individual support headers as well.

![IMG_7662](https://github.com/user-attachments/assets/11ebb74f-a361-4c4c-b7c9-dba3bba4affc)

Cut off four 4-pin strips from a male header.

![IMG_7697](https://github.com/user-attachments/assets/b1636fb9-55a8-4845-a2b9-788d60ecc4a7)
![IMG_7698](https://github.com/user-attachments/assets/82fc7971-c909-47d8-b5d5-ab604e2658dd)

Flip the main board to the side marked "screen board goes here" and place the four header strips: two on the left and right sides, and two on the bottom side, as shown in the photo above.

![IMG_7700](https://github.com/user-attachments/assets/11457e76-1589-4172-a346-4efd11edb5bf)

Keeping the main board in the same position (with "screen board goes here" facing up), sandwich the previously assembled screen board on top.
Double-check that it is correctly positioned.

![IMG_7703](https://github.com/user-attachments/assets/a4c7b1b7-83b0-44f6-9cce-e949ebf807e1)

Solder all the headers on both sides while keeping the two boards pressed firmly together.

![IMG_7707](https://github.com/user-attachments/assets/67cfb1eb-e7c9-4bbc-b525-f026e6518491)

Solder all the headers on both sides while keeping the two boards pressed firmly together.

![IMG_7709](https://github.com/user-attachments/assets/78fed8d8-a853-4484-aadc-e220bd4007d9)

Using the flush side cutter, trim the excess length of the soldered pins as much as possible.

Be careful not to damage the PCB traces; a clean trim ensures the enclosure will close properly.

![IMG_7714](https://github.com/user-attachments/assets/f4b2e234-0aa0-4ba1-a2ce-66dc0961f0c7)

On the main board, trim the excess length of the Pro Micro’s pin headers as well.

Be careful not to damage the PCB traces or any nearby components; a clean trim ensures the enclosure will close properly.

![IMG_7712](https://github.com/user-attachments/assets/987b928f-eb54-4839-8028-cfb8a0ad5530)

The internal assembly is complete!

![IMG_7716](https://github.com/user-attachments/assets/5e13078d-aef8-4063-ae00-ae3587a2fa0b)

After making sure the two boards are properly aligned and parallel to each other, you can proceed to flash the firmware.

Connect Arpmini to your computer using a USB-C cable.
A red LED between the main board and the Pro Micro should light up, indicating that the bootloader is ready to receive the firmware.
Download the full [Arpmini](https://github.com/PaoloEstorm/Arpmini) project and open the Arpmini.ino file with the Arduino IDE.
Make sure Arpmini is selected under Tools → Board → Arpmini Core → Arpmini, then click Upload.
During the upload, press the yellow and blue buttons on Arpmini simultaneously to start the EEPROM initialization procedure.
If you miss the timing, you can still initialize the EEPROM by unplugging and reconnecting Arpmini while holding both buttons.
For this first upload, selecting the serial port is not necessary/possible, but for future uploads you will need to choose Arpmini from the list of available serial ports.

![IMG_7720](https://github.com/user-attachments/assets/e186841a-647b-47e8-9d4e-999f45c23eb7)
![IMG_7727](https://github.com/user-attachments/assets/537b330b-dbcc-40eb-8e08-a704e3a369d0)
![IMG_7729](https://github.com/user-attachments/assets/2981ef82-cee0-4627-8057-78668a46a3c2)
![IMG_7730](https://github.com/user-attachments/assets/dc0bfbf7-5828-45f5-a514-f3a7d7a30472)
![IMG_7731](https://github.com/user-attachments/assets/bc4d3b51-80ed-4a1d-9ac3-41e64aefe49b)
![IMG_7734](https://github.com/user-attachments/assets/c1323db4-8425-4111-8c77-7adec7176eba)
![IMG_7737](https://github.com/user-attachments/assets/a486e447-415a-4313-9520-ea652cf4cbe6)
![IMG_7739](https://github.com/user-attachments/assets/9c2423d8-51bc-42ef-a478-fd3f135873e2)
![IMG_7740](https://github.com/user-attachments/assets/edb1ea13-3468-4977-8b5d-f376edd85b41)
![IMG_7741](https://github.com/user-attachments/assets/2293a4f5-1f1e-44df-9088-aa001ef058a6)
![IMG_7746](https://github.com/user-attachments/assets/4d4d7361-0c79-475a-bc69-44607de547a0)
![IMG_7743](https://github.com/user-attachments/assets/78de7b83-86be-4600-9167-77b5e246dfad)
![IMG_7744](https://github.com/user-attachments/assets/d6dcc56c-a0bc-44a7-b62b-468a843229d0)
![IMG_7748](https://github.com/user-attachments/assets/b718567b-5d1c-443e-a9b2-a7956eafc401)
![IMG_8023](https://github.com/user-attachments/assets/4e788e25-81f3-47b9-bcca-e37a082cc948)
![IMG_8024](https://github.com/user-attachments/assets/5634e9f3-d043-4f46-a627-b255b7435f5b)
![IMG_8025](https://github.com/user-attachments/assets/d4ee9466-6e72-46c0-a862-cf5731b6f6be)
![IMG_8027](https://github.com/user-attachments/assets/3819b711-995e-4037-b7c3-d96daf123b3a)
![IMG_8026](https://github.com/user-attachments/assets/7ef160eb-d8d6-415f-baaf-70a2b16bb54f)
![IMG_8029](https://github.com/user-attachments/assets/f7b39c10-1613-40fc-a026-46b1098142f5)

