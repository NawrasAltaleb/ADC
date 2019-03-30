#ifndef REF_SRC_H
#define REF_SRC_H

#include <systemc-ams>

SCA_TDF_MODULE(parameters)
{
	parameters(sc_core::sc_module_name nm):t_step(sca_core::sca_time(100, sc_core::SC_NS)) {
		initial_config_loop = 1;
		checking_CH_reg_loop = 0;
	}

private:
	void set_attributes() {
		this->set_timestep(t_step);
		accept_attribute_changes();
	}

	void processing() {
		double t = this->get_time().to_seconds();
		bool temp_bits[32] = {false};
		int temp_address = 0x40038000;
		int temp_cmd = 0;

		if (t <= 0.000001) {	//in this period we only send the configuration sequence for all need to be written registers starting with a reset and ending with a begin conversion
			if (initial_config_loop == 1)	// ADC_CR (reset)
			{
				temp_cmd = tlm::TLM_WRITE_COMMAND;
				temp_address = ADC_CR_;
				temp_bits[0] = true;//Resets the ADC
				initial_config_loop++;
				write_trans_parameteres(temp_cmd, temp_address, temp_bits);
			}
			else if (initial_config_loop == 2)	// ADC_MR
			{
				temp_cmd = tlm::TLM_WRITE_COMMAND;
				temp_address = ADC_MR_;
				temp_bits[ADC_MR_LOWRES_0] = false;//LOWRES(0 => 12bits)
				temp_bits[ADC_MR_PRESCAL_0] = true;//PRESCAL(00001001 => ADCClock/20)
				temp_bits[ADC_MR_PRESCAL_3] = true;//PRESCAL(00001001 => ADCClock/20)
				temp_bits[ADC_MR_STARTUP_1] = true;//STARTUP(0110 => 96 ADCClock)
				temp_bits[ADC_MR_STARTUP_2] = true;//STARTUP(0110 => 96 ADCClock)
				temp_bits[ADC_MR_SETTLING_1] = true;//SETTLING(10 => 9 ADCClock)
				temp_bits[ADC_MR_ANACH_0] = true;//ANACH(1 => different gain and offset for different channels)
				temp_bits[ADC_MR_TRACKTIM_0] = true;//TRACKTIM(0101 => 6 ADCClock)
				temp_bits[ADC_MR_TRACKTIM_2] = true;//TRACKTIM(0101 => 6 ADCClock)
				temp_bits[ADC_MR_TRANSFER_0] = true;//TRANSFER(11 => 9 ADCClock)
				temp_bits[ADC_MR_TRANSFER_1] = true;//TRANSFER(11 => 9 ADCClock)
				initial_config_loop++;
				write_trans_parameteres(temp_cmd, temp_address, temp_bits);
			}
			else if (initial_config_loop == 3)	// ADC_CHER (Channel enable register)
			{
				temp_cmd = tlm::TLM_WRITE_COMMAND;
				temp_address = ADC_CHER_;
				temp_bits[0] = true;
				temp_bits[3] = true;
				temp_bits[5] = true;
				temp_bits[7] = true;
				temp_bits[15] = true;
				initial_config_loop++;
				write_trans_parameteres(temp_cmd, temp_address, temp_bits);
			}
			else if (initial_config_loop == 4)	// ADC_CHDR (Channel disable register)
			{
				temp_cmd = tlm::TLM_WRITE_COMMAND;
				temp_address = ADC_CHDR_;
				for(int i=15; i>=0;i--)
					temp_bits[i] = true;
				temp_bits[0] = false;
				temp_bits[3] = false;
				temp_bits[5] = false;
				temp_bits[7] = false;
				temp_bits[15] = false;
				initial_config_loop++;
				write_trans_parameteres(temp_cmd, temp_address, temp_bits);
			}
			else if (initial_config_loop == 5)	// ADC_CGR (Channel Gain register)
			{
				temp_cmd = tlm::TLM_WRITE_COMMAND;
				temp_address = ADC_CGR_;
				temp_bits[7] = true;// CH3_GAIN(10 => 2)
				temp_bits[10] = true;// CH5_GAIN(11 => 4)
				temp_bits[11] = true;// CH5_GAIN(11 => 4)
				initial_config_loop++;
				write_trans_parameteres(temp_cmd, temp_address, temp_bits);
			}
			else if (initial_config_loop == 6)	// ADC_COR (Channel Offset register) 2bits
			{
				temp_cmd = tlm::TLM_WRITE_COMMAND;
				temp_address = ADC_COR_;
				for(int i=15; i>=0;i--)
					temp_bits[i] = true;
				initial_config_loop++;
				write_trans_parameteres(temp_cmd, temp_address, temp_bits);
			}
			else if (initial_config_loop == 7)	// ADC_CR (configured and ready: begin)
			{
				temp_cmd = tlm::TLM_WRITE_COMMAND;
				temp_address = ADC_CR_;
				temp_bits[ADC_CR_START_0]=true;
				initial_config_loop++;
				write_trans_parameteres(temp_cmd, temp_address, temp_bits);
			}
			else{	//not sending anything so just read the channel status register
				temp_cmd = tlm::TLM_READ_COMMAND;
				temp_address = ADC_CHSR_;
				initial_config_loop = 8;
				write_trans_parameteres(temp_cmd, temp_address, temp_bits);
			}
		}
		else if (t >= 0.000001 && t <= 0.000001+(0.0000001*16*1)){//1 round of checking the channel data registers
			temp_cmd = tlm::TLM_READ_COMMAND;
			temp_address = ADC_CDR0_ + checking_CH_reg_loop;
			checking_CH_reg_loop += 4;
			if(checking_CH_reg_loop > (ADC_CDR15_ - ADC_CDR0_))
				checking_CH_reg_loop = 0;
			write_trans_parameteres(temp_cmd, temp_address, temp_bits);
		}
		///////////////// a new period of test with new values for the ADC_MR
		else if (t >= 0.0004 && t <= 0.0004+(0.0000001)){// stop the ADC and restart again to see the effect of stopping the entire cell
				temp_cmd = tlm::TLM_WRITE_COMMAND;
				temp_address = ADC_CR_;
				temp_bits[1] = false;//stop the ADC
				write_trans_parameteres(temp_cmd, temp_address, temp_bits);
		}
		else if (t >= 0.0004 && t <= 0.0004+(0.0000004)){// sitting new values for the ADC_MR
				temp_cmd = tlm::TLM_WRITE_COMMAND;
				temp_address = ADC_MR_;
				temp_bits[ADC_MR_LOWRES_0] = false;//LOWRES(0 => 12bits)
				temp_bits[ADC_MR_PRESCAL_2] = true;//PRESCAL(00000100 => ADCClock/10)
				temp_bits[ADC_MR_STARTUP_0] = true;//STARTUP(0011 => 24 ADCClock)
				temp_bits[ADC_MR_STARTUP_1] = true;//STARTUP(0011 => 24 ADCClock)
				temp_bits[ADC_MR_SETTLING_0] = true;//SETTLING(01 => 5 ADCClock)
				temp_bits[ADC_MR_ANACH_0] = false;//ANACH(0 => same gain and offset for different channels (CH0_GAIN & CH0_OFFSET))
				temp_bits[ADC_MR_TRACKTIM_1] = true;//TRACKTIM(0010 => 3 ADCClock)
				temp_bits[ADC_MR_TRANSFER_0] = true;//TRANSFER(01 => 5 ADCClock)
				write_trans_parameteres(temp_cmd, temp_address, temp_bits);
		}
		else if (t >= 0.0004 && t <= 0.0004+(0.0000008)){// start the ADC and see the effect of the STARTUP time
				temp_cmd = tlm::TLM_WRITE_COMMAND;
				temp_address = ADC_CR_;
				temp_bits[1] = true;//start the ADC
				write_trans_parameteres(temp_cmd, temp_address, temp_bits);
		}
		//////////////// reading the CDRs repeatedly with a phase of rest
		else if (t >= 0.0005){//after this period just keep reading the channels data registers
			if(std::fmod(t,1e-6)<1e-7){//doing a register read every 1 US to avoid excessive operation
			temp_cmd = tlm::TLM_READ_COMMAND;
			temp_address = ADC_CDR0_ + checking_CH_reg_loop;
			checking_CH_reg_loop += 4;
			if(checking_CH_reg_loop > (ADC_CDR15_ - ADC_CDR0_))
					checking_CH_reg_loop = 0;
			write_trans_parameteres(temp_cmd, temp_address, temp_bits);
			}
		}
	}

	void write_trans_parameteres(int t_cmd, int t_add, bool t_bits[]) {
		trans_initiator.reg_cmd = t_cmd;
		trans_initiator.reg_address = t_add;
		for(int i=0;i<32;i++)
				trans_initiator.reg_data[i] = t_bits[i];
	}

	sca_time t_step;
	int initial_config_loop;
	int checking_CH_reg_loop;
};

#endif /* REF_SRC_H_ */
