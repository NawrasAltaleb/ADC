#ifndef _TDF_TO_DE_H
#define _TDF_TO_DE_H

template<class T>
class tdf_to_de: public sca_tdf::sca_module {
public:
	sca_tdf::sca_in<T> in;   // input Port
	sca_tdf::sc_out<T> out;  // output Port

	tdf_to_de(sc_module_name n) {}

private:
	void set_attributes() {
		accept_attribute_changes();
	}

	void processing() {
		out.write(in.read());
	}
};

#endif
