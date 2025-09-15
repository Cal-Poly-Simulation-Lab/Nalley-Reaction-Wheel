/*
This NanoJ Example Code is based on our experience with typical user requirements in a wide range
of industrial applications and is provided without guarantee regarding correctness and completeness.
It serves as general guidance and should not be construed as a commitment of Nanotec to guarantee its
applicability to the customer application without additional tests under the specific conditions
and � if and when necessary � modifications by the customer. 

The responsibility for the applicability and use of the NanoJ Example Code in a particular
customer application lies solely within the authority of the customer.
It is the customer's responsibility to evaluate, investigate and decide,
whether the Example Code is valid and suitable for the respective customer application, or not.
Defects resulting from the improper handling of devices and modules are excluded from the warranty.
Under no circumstances will Nanotec be liable for any direct, indirect, incidental or consequential damages
arising in connection with the Example Code provided. In addition, the regulations regarding the
liability from our Terms and Conditions of Sale and Delivery shall apply.
*/

// Instructions: 
// When the programm is running the state machine will switch to state "Ready to switch on" 
// The statemachine will switch to "operation enabled" when Input 1 is high
// The Velocity is set using the Analog-Input

// You can map frequently used objects to be able to read or write them
// using In.* and Out.*. Here we map the object 6041:00 as "In.StatusWord".
map U16 Controlword as inout 0x6040:00
map U16 Statusword as input 0x6041:00
map U32 Inputs as input 0x60FD:00
map U32 Outputs as inout 0x60FE:01
map S08 ModesOfOperation as output 0x6060:00
map S08 ModesOfOperationDisplay as input 0x6061:00
map S16 AnalogInput as input 0x3220:01
map S16 TargetTorque as inout 0x6071:00



// Include the definition of NanoJ functions and symbols
#include "wrapper.h"
#include "nanotec.h"

// The user() function is the entry point of the NanoJ program. It is called
// by the firmware of the controller when the NanoJ program is started.
void user()
{	
	ModesOfOperation(4);								// set the mode of operation to "profile torque"
	
	
//3.1 Enable voltage, set target torque, set max. torque, set torque slope

	
	Shutdown();									// set the state machine to "ReadyToSwitchOn"
	
	od_write(0x6072, 0x00, 1000); 							// set maximum torque to 100% of the rated torque
	
	od_write(0x6087, 0x00, 1000); 							// set torque slope to 100% of the rated torque within 1s
	
	// Max Torque and Offset
	S32 MaxTorque = 1000;
	S32 Offset = 50;
	
//3.2 Use input 5 as "start"
		
	yield(); 
	
	while(true) 
	{
		
		while(DigitalInput(5))
		{		
			// Defining an offset for the analog input. When the analog input is below this offset the motor should stop (Velocity = 0) 
			if(In.AnalogInput < Offset ) 
			{
				InOut.TargetTorque = 0;
			} 
			// If the analog input is above the offset-value, it should scale the velocity up to the MaxVelocity in a linear connection 
			else		
			{
				InOut.TargetTorque = (MaxTorque * (AnalogInput()-Offset)) / (1023-Offset);   		
			}
			
			EnableOperation();
		} 

		Shutdown(); 	
		yield();
	}
}