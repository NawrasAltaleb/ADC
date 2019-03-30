#ifndef TLM_INITIATOR_H_
#define TLM_INITIATOR_H_

//#include "ADC_REGISTERS.h"
#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "systemc.h"
#include <systemc-ams>

tlm_transaction trans_initiator;
//#define _INIT_TRANS
#ifdef _INIT_TRANS
tlm_transaction last_init_trans;
#endif

// my initiator (from the software to access ADC using PV)
struct data_t: sc_module {
	// TLM-2 socket, defaults to 32-bits wide, base protocol
	tlm_utils::simple_initiator_socket<data_t> socket;
	sc_in<bool> clk;
	sc_out< sc_bv<12> > CDR[16];// when it's a read command from PV this will send out the value of the associated register in the transaction

	void thread_process() {
		// TLM-2 generic periodic transaction (clk), reused across calls to b_transport
		tlm::tlm_generic_payload *trans = new tlm::tlm_generic_payload;
		sc_time delay = sc_time(0, SC_NS);

		tlm::tlm_command cmd = static_cast<tlm::tlm_command>(trans_initiator.reg_cmd);
		trans->set_command(cmd);
		trans->set_address(trans_initiator.reg_address + ADC_BASE_ADDRESS);
		trans->set_data_ptr( reinterpret_cast<unsigned char*>(trans_initiator.reg_data));
		trans->set_data_length(DATA_WIDTH);
		trans->set_streaming_width(DATA_WIDTH);
		trans->set_byte_enable_ptr(0); // 0 indicates unused
		trans->set_dmi_allowed(false); // Mandatory initial value
		trans->set_response_status(tlm::TLM_INCOMPLETE_RESPONSE); // Mandatory initial value

		socket->b_transport(*trans, delay);  // Blocking transport call
		// Initiator obliged to check response status and delay
		if (trans->is_response_error()) {
			char txt[100];
			sprintf(txt, "Error from b_transport, response status = %s",
					trans->get_response_string().c_str());
			std::cout<< "Init-trans("<<sc_time_stamp()<<"): TLM-2: "<<txt<<endl;//SC_REPORT_ERROR("TLM-2", txt);
		}
		//how to handle transactions that reads values?
		else if (trans->is_response_ok() && cmd == tlm::TLM_READ_COMMAND) {	//data is available at (adc_data_ts.reg_bits)
			//here is where the signals representing the channels data registers should go out to the main
			if(trans_initiator.reg_address >= ADC_CDR0_ && trans_initiator.reg_address <= ADC_CDR15_){
				sc_bv<12> temp_value;
				for(int i=11;i>=0;i--)
					temp_value[i] = adc_reg[trans_initiator.reg_address].reg_data[i];
				CDR[(trans_initiator.reg_address-ADC_CDR0_)/4].write(temp_value);
			}
		}

#ifdef _INIT_TRANS
		if(trans_initiator.reg_address != last_init_trans.reg_address || *trans_initiator.reg_data != *last_init_trans.reg_data) {
			std::cout << "Init-trans("<<sc_time_stamp()<<"): cmd("<<trans_initiator.reg_cmd<<") add(0x"<<std::hex<<trans_initiator.reg_address<<") data: ";
			for (int i = DATA_WIDTH; i > 0; i--) {
				std::cout << trans_initiator.reg_data[i - 1];
				last_init_trans.reg_data[i-1] = trans_initiator.reg_data[i-1];
			}
			std::cout << endl;
			last_init_trans.reg_address = trans_initiator.reg_address;
		}
#endif
	}

	SC_CTOR(data_t) : socket("socket")  // Construct and name socket
	{
		SC_METHOD(thread_process);
		sensitive << clk.neg();
		dont_initialize();
	}
};

#endif /* TLM_INITIATOR_H_ */
