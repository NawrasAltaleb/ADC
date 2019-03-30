#include <systemc-ams.h>
#include <tuv_ams_library.h>
#include "ADC_REGISTERS.h"
#include "tlm_initiator.h"
#include "tlm_target.h"
#include "ref_src.h"
#include "ADC.h"
#include "tdf_to_de.h"

using namespace TUV_ams_lib::bb;

template<class T, int N>
SCA_TDF_MODULE(drain) {
	sca_tdf::sca_in< T > in[N];
	SCA_CTOR(drain) {}
	void processing() {}
};

int sc_main(int argc, char* argv[]) {

	sc_set_time_resolution(10, SC_NS);	// setting the time resolution

	/////////////////////////////////////////////////// TLM transactions part
		sc_clock MCK("MCK", 100, SC_NS);
		sc_core::sc_signal< sc_bv<12> > CDR[16];//as output from the transaction initiator representing the channels digitized values

		//parameter generator for sine wave//
		parameters ref_src("ref_src");

		//TLM initiator and target//
		data_t *parameters_initiator = new data_t("data_t");
		parameters_initiator->clk(MCK);
		for(int i=0;i<16;i++)
			parameters_initiator->CDR[i](CDR[i]);


		data_r *parameters_target = new data_r("data_r");
		parameters_initiator->socket.bind(parameters_target->socket);



		/////////////////////////////////////////////////// ADC part

		sca_tdf::sca_signal<bool> ADCClock;//Control Logic create the ADCClock using PRESCAL relative to the MCK
		sca_tdf::sca_signal<bool> RUNNING;//the ADC will run for a start up period before being ready
		sca_tdf::sca_signal<bool> READY;//the ADC conversion starts with this
		sca_tdf::sca_signal<bool> HOLD;//a signal for the S_H module to hold the value after being done with TRACKING and TRANSFER it
		sca_tdf::sca_signal<bool> WRITE;//a signal to the allocator module to know where to write the converted value with the help of WRITE_SEL
		sca_tdf::sca_signal<int> MUL_SEL;//a signal to specify which channel to select at the multiplexer
		sca_tdf::sca_signal<int> WRITE_SEL;//a signal to specify which channel to write to at the allocator (follows the value of the MUL_SEL)
		sca_tdf::sca_signal<double> GAIN;
		sca_tdf::sca_signal<double> OFFSET;

		//Control Logic takes the parameters (registers) from TLM target and extract the useful signals and values for submodules
		Control_Logic adc_Control_Logic("adc_Control_Logic");
		adc_Control_Logic.outADCClock(ADCClock);
		adc_Control_Logic.outRUNNING(RUNNING);
		adc_Control_Logic.outREADY(READY);
		adc_Control_Logic.outHOLD(HOLD);
		adc_Control_Logic.outWRITE(WRITE);
		adc_Control_Logic.outMUL_SEL(MUL_SEL);
		adc_Control_Logic.outWRITE_SEL(WRITE_SEL);
		adc_Control_Logic.outGAIN(GAIN);
		adc_Control_Logic.outOFFSET(OFFSET);
		adc_Control_Logic.set_timestep(100, sc_core::SC_NS);


		//////////////////// Generate input signals ////////////////////
		sca_tdf::sca_signal<double> zero_data_tdf;//for Not used channels
		sca_tdf::sca_signal<double> ADVREF_33_tdf;
		sca_tdf::sca_signal<double> AD3_sig_sin_tdf;
		sca_tdf::sca_signal<double> AD5_sig_cos_tdf;
		sca_tdf::sca_signal<double> AD7_sig_saw_tdf;
		sca_tdf::sca_signal<double> ADVREF_1_tdf;

		sine zero_data_src("void_data_src", 0, 0, 1.0, 0.0, false, false, 1);
		zero_data_src.out(zero_data_tdf);		zero_data_src.set_timestep(200, sc_core::SC_NS);
		sine ADVREF_S("ADVREF", 0, 3.3, 1.57, 0.0, false, false, 1);
		ADVREF_S.out(ADVREF_33_tdf);			ADVREF_S.set_timestep(200, sc_core::SC_NS);
		sine Sine("Sine", 100, 2, 0.0, 0.0, false, false, 1);
		Sine.out(AD3_sig_sin_tdf);				Sine.set_timestep(200, sc_core::SC_NS);
		sine Cos("Cos", 500, 2.5, 1.57, 0.0, false, false, 1);
		Cos.out(AD5_sig_cos_tdf);				Cos.set_timestep(200, sc_core::SC_NS);
		saw_gen Saw("Saw", 50, 2.5, 0.0, 1);
		Saw.out(AD7_sig_saw_tdf);				Saw.set_timestep(200, sc_core::SC_NS);
		sine ADVREF_end_temp_S("ADVREF_end_temp", 0, 1, 1.57, 0.0, false, false, 1);
		ADVREF_end_temp_S.out(ADVREF_1_tdf);	ADVREF_end_temp_S.set_timestep(200, sc_core::SC_NS);

		////////// create a drain for the tdf input signals to avoid creating many dataflow clusters
		drain<double,6> drain_tdf_signals("drain_tdf_signals");
		drain_tdf_signals.in[0](zero_data_tdf);
		drain_tdf_signals.in[1](ADVREF_33_tdf);
		drain_tdf_signals.in[2](AD3_sig_sin_tdf);
		drain_tdf_signals.in[3](AD5_sig_cos_tdf);
		drain_tdf_signals.in[4](AD7_sig_saw_tdf);
		drain_tdf_signals.in[5](ADVREF_1_tdf);


		///////////////// the link between input signals and ADC cell should be through de signals in order not to let the ADC_cell effect the sampling of the signals
		sc_core::sc_signal<double> zero_data_de;//for Not used channels
		sc_core::sc_signal<double> ADVREF_33_de;
		sc_core::sc_signal<double> AD3_sig_sin_de;
		sc_core::sc_signal<double> AD5_sig_cos_de;
		sc_core::sc_signal<double> AD7_sig_saw_de;
		sc_core::sc_signal<double> ADVREF_1_de;

		tdf_to_de<double>* tdf_2_de_signals[6];
		for(int i=0;i<6;i++){
			char name[50];
			sprintf(name,"tdf_2_de_signal%i",i);
			tdf_2_de_signals[i] = new tdf_to_de<double>(name);
		}
		tdf_2_de_signals[0]->in(zero_data_tdf);	tdf_2_de_signals[0]->out(zero_data_de);
		tdf_2_de_signals[1]->in(ADVREF_33_tdf); tdf_2_de_signals[1]->out(ADVREF_33_de);
		tdf_2_de_signals[2]->in(AD3_sig_sin_tdf); tdf_2_de_signals[2]->out(AD3_sig_sin_de);
		tdf_2_de_signals[3]->in(AD5_sig_cos_tdf); tdf_2_de_signals[3]->out(AD5_sig_cos_de);
		tdf_2_de_signals[4]->in(AD7_sig_saw_tdf); tdf_2_de_signals[4]->out(AD7_sig_saw_de);
		tdf_2_de_signals[5]->in(ADVREF_1_tdf); tdf_2_de_signals[5]->out(ADVREF_1_de);


		//////////////////// multiplexer stage ////////////////////
		sca_tdf::sca_signal<double> multiplexer_out;

		adc_multiplexer<double,16> my_multiplexer("my_multiplexer");
		my_multiplexer.ADCClock(ADCClock);
		my_multiplexer.sel(MUL_SEL);
		my_multiplexer.AD[0](ADVREF_33_de);
		my_multiplexer.AD[1](zero_data_de);//connecting the Not used channels to nothing
		my_multiplexer.AD[2](zero_data_de);//connecting the Not used channels to nothing
		my_multiplexer.AD[3](AD3_sig_sin_de);
		my_multiplexer.AD[4](zero_data_de);//connecting the Not used channels to nothing
		my_multiplexer.AD[5](AD5_sig_cos_de);
		my_multiplexer.AD[6](zero_data_de);//connecting the Not used channels to nothing
		my_multiplexer.AD[7](AD7_sig_saw_de);
		for(int i=8;i<15;i++)//connecting the Not used channels to nothing
				my_multiplexer.AD[i](zero_data_de);
		my_multiplexer.AD[15](ADVREF_1_de);
		my_multiplexer.out(multiplexer_out);


		//////////////////// OFFSET stage ////////////////////
		sca_tdf::sca_signal<double> offset_out;

		adc_offset<double> my_offset("my_offset");
		my_offset.ADCClock(ADCClock);
		my_offset.ADVREF(ADVREF_33_de);
		my_offset.offset(OFFSET);
		my_offset.in(multiplexer_out);
		my_offset.out(offset_out);

		//////////////////// Gain stage ////////////////////
		// this is happening before the S&H module because the channel's signal needs to be offseted and gained before holding it
		sca_tdf::sca_signal<double> pga_out;

		adc_pga<double> my_pga("my_pga");
		my_pga.ADCClock(ADCClock);
		my_pga.gain(GAIN);
		my_pga.in(offset_out);
		my_pga.out(pga_out);

		//////////////////// Sample and Hold stage ////////////////////
		sca_tdf::sca_signal<double> S_H_out;

		adc_S_H<double> my_S_H("my_S_H");
		my_S_H.ADCClock(ADCClock);
		my_S_H.hold(HOLD);
		my_S_H.in(pga_out);
		my_S_H.out(S_H_out);

		//////////////////// AD Conversion stage ////////////////////
		sca_tdf::sca_signal< sc_bv<12> > adc_out;

		adc<12> my_adc("my_adc", 3.3, 0.0, 0.0);  //TRANSFER as delay on output
		my_adc.in(S_H_out);
		my_adc.out(adc_out);

		//////////////////// Allocate converted value stage ////////////////////
		adc_allocator my_allocator("my_allocator");
		my_allocator.ADCClock(ADCClock);
		my_allocator.write(WRITE);
		my_allocator.write_sel(WRITE_SEL);
		my_allocator.in(adc_out);

		/////////////////// Create the tracking files ////////////////
		sca_util::sca_trace_file* adc_inner_signals = sca_util::sca_create_vcd_trace_file("adc_inner_signals.vcd");
		sca_util::sca_trace(adc_inner_signals, MCK, "00_MCK");
		sca_util::sca_trace(adc_inner_signals, ADCClock, "01_ADCClock");
		sca_util::sca_trace(adc_inner_signals, RUNNING, "02_RUNNING");
		sca_util::sca_trace(adc_inner_signals, READY, "03_READY");
		sca_util::sca_trace(adc_inner_signals, MUL_SEL, "04_MUL_SEL");
		sca_util::sca_trace(adc_inner_signals, OFFSET, "05_OFFSET");
		sca_util::sca_trace(adc_inner_signals, GAIN, "06_GAIN");
		sca_util::sca_trace(adc_inner_signals, HOLD, "07_HOLD");
		sca_util::sca_trace(adc_inner_signals, WRITE_SEL, "08_WRITE_SEL");
		sca_util::sca_trace(adc_inner_signals, WRITE, "09_WRITE");
		/*sca_util::sca_trace(adc_inner_signals, AD3_sig_sin_de, "10_AD3_sig_sin");
		sca_util::sca_trace(adc_inner_signals, AD5_sig_cos_de, "11_AD5_sig_cos");
		sca_util::sca_trace(adc_inner_signals, AD7_sig_saw_de, "12_AD7_sig_saw");
		sca_util::sca_trace(adc_inner_signals, multiplexer_out, "13_multiplexer_out");
		sca_util::sca_trace(adc_inner_signals, offset_out, "14_offset_out");
		sca_util::sca_trace(adc_inner_signals, pga_out, "15_pga_out");
		sca_util::sca_trace(adc_inner_signals, S_H_out, "16_S_H_out");
		sca_util::sca_trace(adc_inner_signals, adc_out, "17_adc_out");*/


		sca_util::sca_trace_file* adc_inner_signals_tab = sca_util::sca_create_tabular_trace_file("adc_inner_signals.dat");
		/*sca_util::sca_trace(adc_inner_signals_tab, MCK, "00_MCK");
		sca_util::sca_trace(adc_inner_signals_tab, ADCClock, "01_ADCClock");
		sca_util::sca_trace(adc_inner_signals_tab, RUNNING, "02_RUNNING");
		sca_util::sca_trace(adc_inner_signals_tab, READY, "03_READY");
		sca_util::sca_trace(adc_inner_signals_tab, MUL_SEL, "04_MUL_SEL");
		sca_util::sca_trace(adc_inner_signals_tab, OFFSET, "05_OFFSET");
		sca_util::sca_trace(adc_inner_signals_tab, GAIN, "06_GAIN");
		sca_util::sca_trace(adc_inner_signals_tab, HOLD, "07_HOLD");
		sca_util::sca_trace(adc_inner_signals_tab, WRITE_SEL, "08_WRITE_SEL");
		sca_util::sca_trace(adc_inner_signals_tab, WRITE, "09_WRITE");*/
		sca_util::sca_trace(adc_inner_signals_tab, AD3_sig_sin_de, "10_AD3_sig_sin");
		sca_util::sca_trace(adc_inner_signals_tab, AD5_sig_cos_de, "11_AD5_sig_cos");
		sca_util::sca_trace(adc_inner_signals_tab, AD7_sig_saw_de, "12_AD7_sig_saw");
		sca_util::sca_trace(adc_inner_signals_tab, multiplexer_out, "13_multiplexer_out");
		sca_util::sca_trace(adc_inner_signals_tab, offset_out, "14_offset_out");
		sca_util::sca_trace(adc_inner_signals_tab, pga_out, "15_pga_out");
		sca_util::sca_trace(adc_inner_signals_tab, S_H_out, "16_S_H_out");
		sca_util::sca_trace(adc_inner_signals_tab, adc_out, "17_adc_out");


		//create track file for the channels registers digitized values
		sca_util::sca_trace_file* CDR_data = sca_util::sca_create_vcd_trace_file("CDR_data.vcd");
		for(int i=0;i<16;i++){
			char name[5];
			sprintf(name,"CDR%i",i);
			sca_util::sca_trace(CDR_data, CDR[i], name);
		}

	/////// running the simulation
	int sim_duration = 10;
	time_t start_time = time(0);
	cout << "Start generating files at time: "<<asctime(localtime(&start_time))<<"\t\tadc_inner_control_signals.vcd\n\t\tadc_inner_data_signals.dat\n\t\tCDR_data.vcd"<< endl;

	sc_start(sim_duration, SC_MS);

	sca_util::sca_close_vcd_trace_file(adc_inner_signals);
	sca_util::sca_close_tabular_trace_file(adc_inner_signals_tab);
	sca_util::sca_close_vcd_trace_file(CDR_data);

	time_t end_time = time(0);
	cout << "Finished generating files at time: "<<asctime(localtime(&end_time))<<"\t\tadc_inner_control_signals.vcd\n\t\tadc_inner_data_signals.dat\n\t\tCDR_data.vcd"<< endl;
	cout << "Simulation of "<<sim_duration<<" MS took: "<<difftime (end_time, start_time )<<" seconds."<<endl;
	return 0;
};

