#ifndef ADC_H_
#define ADC_H_

#include <systemc-ams.h>

static const double START_UP_Table[] = {0, 8, 16, 24, 64, 80, 96, 112, 512, 576, 640, 704, 768, 832, 896, 960};
static const double SETTLING_Table[] = {3, 5, 9, 17};
static const double GAIN_Table[] = {1, 1, 2, 4};
// Control_Logic class
class Control_Logic: public sca_tdf::sca_module {
public:
	sca_tdf::sca_out<bool> outADCClock;		// output clk
	sca_tdf::sca_out<bool> outRUNNING;		// output port no need for it but to indicate that adc is processing now (starting with the STARTUP time!!)
	sca_tdf::sca_out<bool> outREADY;		// output port no need for it but to indicate that adc is ready to convert data now (after passing the STARTUP time!!)
	sca_tdf::sca_out<bool> outHOLD;			// output port signal to the S&H module to hold the next sample (between the TRACKING and TRANSFER time!!)
	sca_tdf::sca_out<bool> outWRITE;		// output port signal to the Allocator module to write the next digital sample to the right register in memory (WRITE_SEL)
	sca_tdf::sca_out<int> outMUL_SEL;		// output port channel number to pass through from at the multiplexer
	sca_tdf::sca_out<int> outWRITE_SEL;		// output port channel number to write to register in memory (follows MUL_SEL)
	sca_tdf::sca_out<double> outGAIN;
	sca_tdf::sca_out<double> outOFFSET;		// output port to specify the offset "relative" value (WITHOUT the Vref value!!!) (shouldn't link the Vref signal to the logic)

private:
	bool ADCClock_level;
	double t_sampling;
	double var_RUNNING;

	double var_TRANSFER;
	double var_TRACKTIM;
	bool var_ANACH;
	double var_SETTLING;
	double var_STARTUP;	double counter_STARTUP;
	double var_PRESCAL;
	bool var_LOWRES;

	int var_MUL_SEL;//enabled channel at the multiplexer
	double counter_CH_SEL;//counts for a period of time = Tracking time + Transfer time
	bool var_hold;//to tell the S_H when to hold the value from the analog input (after TRACK time and for TRANSFER period)
	int var_WRITE_SEL;//channel register reference after the adc to know where to write the converted value
	bool var_write;

	double var_GAIN;//to check if gain changes between selected channels which requires to add a settling time
	double pre_GAIN;
	double var_OFFSET;//to check if offset changes between selected channels which requires to add a settling time
	double pre_OFFSET;

	void initialize() {
		t_sampling = get_timestep().to_seconds();
	}

	void set_attributes() {
		does_attribute_changes();
		accept_attribute_changes();
	}

	void change_attributes() {
		double t_current = this->get_time().to_seconds();
		double t_pos = std::fmod(t_current, t_sampling);
		if (t_pos < var_PRESCAL) //ADCClock = MCK / ( (PRESCAL+1) * 2 )
			request_next_activation(((var_PRESCAL + 1)) * 100,sc_core::SC_NS);//i need to get rid of the (*2) if i want to run with concurrent number of MCKs
	}

	void processing() {
		if (adc_reg[ADC_CR_].reg_data[0] || !adc_reg[ADC_CR_].reg_data[1]) {
			var_RUNNING = 0;
			outRUNNING.write(false);
			outREADY.write(false);
		} // it's like I'm freezing the entire core when I'm resetting or when I'm not running
		else {
			if(ADCClock_level == false){//the logic control runs at (falling edge) so that the control signals will be ready
									//at the modules before the (rising edge) arrive at them
			}//if(last_ADCClock_level == true)
			else{//here process registers (what to read and extract and what to write) maybe send the signal to tell where to write the converted value here
				//and send the logic signals to drive the core
				sc_bv<DATA_WIDTH> temp_reg;
				temp_reg = adc_reg[ADC_MR_].reg_data;

				var_TRANSFER =	((temp_reg.range(29,28).to_uint()) * 2) + 3;//Transfer Period = (TRANSFER * 2 + 3) ADCClock periods
				var_TRACKTIM =	temp_reg.range(27,24).to_uint()+1;//Tracking Time = (TRACKTIM + 1) * ADCClock periods
				var_ANACH =		temp_reg.range(23,23).to_uint();
				var_SETTLING =	SETTLING_Table[temp_reg.range(21,20).to_uint()];//from the lookup table specified above
				var_STARTUP =	START_UP_Table[temp_reg.range(19,16).to_uint()];//from the lookup table specified above
				var_PRESCAL =	temp_reg.range(15,8).to_uint();//(PRESCAL: for my general ADCClock!!!!!)
				var_LOWRES =	temp_reg.range(4,4).to_uint();

				if(adc_reg[ADC_CR_].reg_data[1] && var_RUNNING==0)//the first trigger to run
				{
					var_RUNNING = 1;
					counter_STARTUP = var_STARTUP;
					outRUNNING.write(true);
					var_MUL_SEL = var_WRITE_SEL = 0;//reset the selected channel to CH0
					counter_CH_SEL = 0;
				}//the first trigger to run
				else if (counter_STARTUP != 0)//the startup stage
				{
					counter_STARTUP--;
				}//the startup stage
				else if(counter_STARTUP == 0)//now the adc runs
				{
					outREADY.write(true);//keep it as ready
					if(var_hold){//checking this before anything else to make sure it only sets for one ADCClock cycle
						var_hold = false;
						outHOLD.write(var_hold);
					}
					if(var_write){//checking this before anything else to make sure it only sets for one ADCClock cycle
						var_write = false;
						outWRITE.write(var_write);
					}

					//////////switching to next channel to convert
					if (counter_CH_SEL == 0){
						/////////// write converted value
						var_WRITE_SEL = var_MUL_SEL;//getting the already converted channel number in order to write it to register memory
						var_write = true;
						outWRITE.write(var_write);
						outWRITE_SEL.write(var_WRITE_SEL);
						/////////// getting the number of the next channel to convert
						for (int i = 1; i < DATA_WIDTH / 2; i++) {//going through channel status reg
							if (adc_reg[ADC_CHSR_].reg_data[(i + var_MUL_SEL) % (DATA_WIDTH / 2)]) {
								var_MUL_SEL = (i + var_MUL_SEL)	% (DATA_WIDTH / 2);
								outMUL_SEL.write(var_MUL_SEL);
								break;
							}//found next enabled channel
						}//selecting next channel
						///////////////get this channel's gain and offset
						pre_GAIN = var_GAIN;//store the previous channel's gain
						pre_OFFSET = var_OFFSET;//store the previous channel's offset
						if (var_ANACH) {//different gains and offsets for different channels
							sc_bv<2> temp_gain;	//read the gain bits for the channel
							temp_gain[0] = adc_reg[ADC_CGR_].reg_data[var_MUL_SEL * 2 + 0];
							temp_gain[1] = adc_reg[ADC_CGR_].reg_data[var_MUL_SEL * 2 + 1];
							//simple method to select the gain without considering DIFF
							if (temp_gain.range(1, 0) == "11")
								var_GAIN = 4;
							else if (temp_gain.range(1, 0) == "10")
								var_GAIN = 2;
							else
								var_GAIN = 1;

							if (adc_reg[ADC_COR_].reg_data[var_MUL_SEL])//select the offset value for this channel
								var_OFFSET = (var_GAIN - 1) / 2;
							else
								var_OFFSET = 0;
						}//if(var_ANACH)
						else {//if(var_ANACH==0) -> CH0 gain and offset apply for all channels
							sc_bv<2> temp_gain;	//read the gain bits for the channel
							temp_gain[0] = adc_reg[ADC_CGR_].reg_data[0 * 2 + 0];
							temp_gain[1] = adc_reg[ADC_CGR_].reg_data[0 * 2 + 1];
							//simple method to select the gain without considering DIFF
							if (temp_gain.range(1, 0) == "11")
								var_GAIN = 4;
							else if (temp_gain.range(1, 0) == "10")
								var_GAIN = 2;
							else
								var_GAIN = 1;

							if (adc_reg[ADC_COR_].reg_data[0])//select the offset value for this channel
								var_OFFSET = (var_GAIN - 1) / 2;
							else
								var_OFFSET = 0;
						}//if(var_ANACH==0)
						outGAIN.write(var_GAIN);
						outOFFSET.write(var_OFFSET);///////////// i think it should be offset = (G-1)*Vref/(2G) if we are converting on a scal from [-Vref , Vref]
						///////////////checking if this channels offset and gain differs from the previous channel then add settling time to the counter_CH_CEL
						if(var_GAIN != pre_GAIN || var_OFFSET != pre_OFFSET)
							counter_CH_SEL = var_SETTLING + var_TRACKTIM + var_TRANSFER;
						else
							counter_CH_SEL = var_TRACKTIM + var_TRANSFER;
					}//if (counter_CH_SEL == 0)
					else if(counter_CH_SEL == var_TRANSFER){//done with Tracking the channel and now should hold the value
						var_hold = true;
						outHOLD.write(var_hold);
					}//if(counter_CH_SEL == var_TRANSFER)
					counter_CH_SEL--;
				}//if(counter_STARTUP == 0)
			}//if(last_ADCClock_level == true)
			outADCClock.write(!ADCClock_level);
			ADCClock_level = !ADCClock_level;//swap to the other clk level
		}//else (not resetting)
	}

public:
	// The constructor of the module.
	Control_Logic(sc_core::sc_module_name n): outADCClock("Control_Logic_outADCClock"), outRUNNING("Control_Logic_outRUNNING"), outREADY("Control_Logic_outREADY"),
	outHOLD("Control_Logic_outHOLD"), outWRITE("Control_Logic_outWRITE"), outMUL_SEL("Control_Logic_outMUL_SEL"), outWRITE_SEL("Control_Logic_outWRITE_SEL"),
	outGAIN("Control_Logic_outGAIN"), outOFFSET("Control_Logic_outOFFSET")
	{
		ADCClock_level = false;
		t_sampling = 0;

		var_RUNNING = 0;
		var_TRANSFER = 0;
		var_TRACKTIM = 0;
		var_ANACH = false;
		var_SETTLING = 0;
		var_STARTUP = 0;counter_STARTUP=0;
		var_PRESCAL = 0;
		var_LOWRES = false;
		var_MUL_SEL = 0;
		counter_CH_SEL = 0;
		var_WRITE_SEL = 0;
		var_hold = false;
		var_write = false;
		var_GAIN = 0;
		pre_GAIN = 0;
		var_OFFSET = 0;
		pre_OFFSET = 0;
	}
};// module Control_Logic

// 16 Channels Multiplexer (should be adjusted to become with generic width)
template<class T, int N>
SCA_TDF_MODULE(adc_multiplexer) {
public:
	sca_tdf::sca_in<bool> ADCClock;	// input clk
	sca_tdf::sca_in<int> sel;			// input channel select number
	sca_tdf::sc_in<T> AD[N];		// input ports
	sca_tdf::sca_out<T> out;		// output port

private:
	void set_attributes() {
		accept_attribute_changes();}

	void processing() {
		if (ADCClock.read()) //work on rising edge
			out.write(AD[sel.read()].read());
	}

public:
	adc_multiplexer(sc_core::sc_module_name n) : ADCClock("adc_multiplexer_ADCClock"), sel("adc_multiplexer_sel"), out("adc_multiplexer_out"){}
};// MODULE(adc_multiplexer)

// Generic OFFSET class
template<class T>
class adc_offset: public sca_tdf::sca_module {
public:
	sca_tdf::sca_in<bool> ADCClock;	// input clk
	sca_tdf::sc_in<T> ADVREF;		// input ADVREF
	sca_tdf::sca_in<T> offset;		// input offset
	sca_tdf::sca_in<T> in;			// input port
	sca_tdf::sca_out<T> out;		// output port

private:
	void set_attributes() {
		accept_attribute_changes();}

	void processing() {
		if (ADCClock.read()) //work on rising edge
			out.write(in.read()+(offset.read() * ADVREF.read()));
	}

public:
	// The constructor of the module.
	adc_offset(sc_core::sc_module_name n) : ADCClock("adc_offset_ADCClock"), ADVREF("adc_offset_ADVREF"),
	offset("adc_offset_offset"), in("adc_offset_in"), out("adc_offset_out") {}
};// module offset

// Generic PGA class
template<class T>
class adc_pga: public sca_tdf::sca_module {
public:
	sca_tdf::sca_in<bool> ADCClock;	// input clk
	sca_tdf::sca_in<T> gain;		// input gain
	sca_tdf::sca_in<T> in;			// input port
	sca_tdf::sca_out<T> out;		// output port

private:
	void set_attributes() {
			accept_attribute_changes();}

	void processing() {
		if (ADCClock.read()) //work on rising edge
			out.write(in.read()*gain.read());
	}

public:
	// The constructor of the module.
	adc_pga(sc_core::sc_module_name n): ADCClock("adc_pga_ADCClock"), gain("adc_pga_gain"), in("adc_pga_in"), out("adc_pga_out"){}
};// module pga

// Generic S/H class
template<class T>
class adc_S_H: public sca_tdf::sca_module {
public:
	sca_tdf::sca_in<bool> ADCClock;	// input clk
	sca_tdf::sca_in<bool> hold;		// input hold signal
	sca_tdf::sca_in<T> in;               // input port
	sca_tdf::sca_out<T> out;             // output port

private:
	void set_attributes() {
		accept_attribute_changes();}

	void processing() {
		if (ADCClock.read()) //work on rising edge
			if(hold.read()) //only transfer the input when HOLD signal is on
				out.write(in.read());
	}

public:
	// The constructor of the module.
	adc_S_H(sc_core::sc_module_name n) : ADCClock("adc_S_H_ADCClock"), hold("adc_S_H_gain"), in("adc_S_H_in"), out("adc_S_H_out"){}
};// module S_H

// then comes the adc module from the tuv_ams_library

//#define DISPLAY_ALLOCATION
// Generic allocator class
class adc_allocator: public sca_tdf::sca_module {
public:
	sca_tdf::sca_in<bool> ADCClock;	// input clk
	sca_tdf::sca_in<bool> write;	// input write signal
	sca_tdf::sca_in<int> write_sel;	// input register number (to write to it)
	sca_tdf::sca_in< sc_bv<12> > in;	// input port with adjustable width

private:
	void set_attributes() {
		accept_attribute_changes();}

	void processing() {
		if (ADCClock.read()){ //work on rising edge
			if(write.read()){ //only write the input digital value when WRITE signal is on
				sc_bv<12> temp = in.read();
				if(adc_reg[ADC_MR_].reg_data[4]){// LOWRRS: 10 bits resolution
					if(temp[11])//negative number
						temp = ("1","1",temp.range(11,2));
					else//positive number
						temp = ("0","0",temp.range(11,2));
				}
				for(int i=11;i>=0;i--)
					adc_reg[ADC_CDR0_ + write_sel.read()*4].reg_data[i] = temp[i];

#ifdef DISPLAY_ALLOCATION
				std::cout<<"t_current = "<<this->get_time().to_seconds()<<" adc_out "<<temp.range(11,0)<<"-  adc_reg["<<write_sel.read()<<"]: ";
				for (int i = DATA_WIDTH-1; i >= 0; i--)
						std::cout<<adc_reg[ADC_CDR0_ + write_sel.read()*4].reg_data[i];
				std::cout<<endl;
#endif
			}
		}
	}

public:
	// The constructor of the module.
	adc_allocator(sc_core::sc_module_name n) : ADCClock("adc_allocator_ADCClock"), write("adc_allocator_write"), write_sel("adc_allocator_write_sel"), in("adc_allocator_in"){}
};// module allocator

#endif /* ADC_H_ */
