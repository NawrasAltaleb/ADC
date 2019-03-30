#ifndef TLM_TARGET_H_
#define TLM_TARGET_H_

//#include "ADC_REGISTERS.h"
#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "systemc.h"
#include <systemc-ams>

tlm_transaction trans_target;
//#define _TARG_TRANS
#ifdef _TARG_TRANS
tlm_transaction last_targ_trans;
#endif

// my target (from the ADC to communicate registers status to and from PV)
struct data_r: sc_module {
	// TLM-2 socket, defaults to 32-bits wide, base protocol
	tlm_utils::simple_target_socket<data_r> socket;

	SC_CTOR(data_r) :
			socket("socket") {
		// Register callback for incoming b_transport interface method call
		socket.register_b_transport(this, &data_r::b_transport);
	}

	// TLM-2 blocking transport method
	virtual void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay) {
		tlm::tlm_command cmd = trans.get_command();
		sc_dt::uint64 adr = trans.get_address() - ADC_BASE_ADDRESS;
		unsigned char* ptr = trans.get_data_ptr();
		unsigned int len = trans.get_data_length();
		unsigned char* byt = trans.get_byte_enable_ptr();
		unsigned int wid = trans.get_streaming_width();

		trans_target.reg_cmd = cmd;
		trans_target.reg_address = adr;
		if (adr < 0x00 || adr > 0xFC)
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
		else if (byt != 0)
			trans.set_response_status(tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE);
		else if (len > DATA_WIDTH || wid < len)//4 bytes
			trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
		else { // correct transaction sent
			   // Obliged to implement read and write commands
			if (cmd == tlm::TLM_READ_COMMAND) {
				if (adr == ADC_CR_ || adr == ADC_CHER_ || adr == ADC_CHDR_)
					trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE); //trying to read a write-only register
				else {
					for (int i = 0; i < DATA_WIDTH; i++)
						trans_target.reg_data[i] = adc_reg[adr].reg_data[i];
					memcpy(ptr, &trans_target.reg_data, len);
					trans.set_response_status(tlm::TLM_OK_RESPONSE);
				}
			} // TLM_READ_COMMAND
			else if (cmd == tlm::TLM_WRITE_COMMAND) {
				if (adr == ADC_CHSR_)
					trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE); //trying to write a read-only register
				else {
					memcpy(&trans_target.reg_data, ptr, len);
					for (int i = 0; i < DATA_WIDTH; i++)
						adc_reg[adr].reg_data[i] = trans_target.reg_data[i];
					trans.set_response_status(tlm::TLM_OK_RESPONSE);
					if (adr == ADC_CR_){//write to ADC_CR
						if (trans_target.reg_data[0]) //reseting all registers to zeros
							for (int i = 4; i < (ADC_MAX_ADDRESS - ADC_BASE_ADDRESS);i++)
								for (int j = 0; j < DATA_WIDTH / 4; j++)
									adc_reg[i].reg_data[j] = false;
					}//write to ADC_CR
					else if (adr == ADC_CHER_){//write to ADC_CHSR and ADC_CHER
						for (int i = DATA_WIDTH-1; i >= 0; i--){
							adc_reg[ADC_CHSR_].reg_data[i] = adc_reg[ADC_CHSR_].reg_data[i] || trans_target.reg_data[i];
							adc_reg[ADC_CHER_].reg_data[i] = trans_target.reg_data[i];}
					}//end process ADC_CHER
					else if (adr == ADC_CHDR_){//write to ADC_CHSR and ADC_CHDR
						for (int i = DATA_WIDTH-1; i >= 0; i--){
							adc_reg[ADC_CHSR_].reg_data[i] = adc_reg[ADC_CHSR_].reg_data[i] && !(trans_target.reg_data[i]);
							adc_reg[ADC_CHDR_].reg_data[i] = trans_target.reg_data[i];}
					}//end process ADC_CHDR
				}//end of (adr != ADC_CHSR_)
			} // TLM_WRITE_COMMAND
		} // correct transaction sent

#ifdef _TARG_TRANS
		if(trans_target.reg_address != last_targ_trans.reg_address || *trans_target.reg_data != *last_targ_trans.reg_data) {
			std::cout<<std::hex<< "Targ-trans("<<sc_time_stamp()<<"): cmd("<<trans_target.reg_cmd<<") add(0x"<<trans_target.reg_address<<") data: ";
			for (int i = DATA_WIDTH; i > 0; i--) {
				std::cout << trans_target.reg_data[i - 1];
				last_targ_trans.reg_data[i-1] = trans_target.reg_data[i-1];
			}
			std::cout << endl;
			last_targ_trans.reg_address = trans_target.reg_address;
		}
#endif
	} //b_transport
};

#endif /* TLM_TARGET_H_ */
