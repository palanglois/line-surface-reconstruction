#ifndef OPTIONPARSER_H
#define OPTIONPARSER_H

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <vector>

#include <typeinfo>

using namespace std;



namespace op{
   
/*
	convert a boolean to a string
*/
string bool2str(bool value){
 return (value)? "true": "false";
}

/*
	convert a string to a boolean
*/
bool str2bool(string value){
 return (value == "true" || value == "TRUE" );
}

/*
	convert a string to an int
*/
int str2int(string value){
 return atoi(value.c_str());
}

/*
	convert a string to a float
*/
float str2float(string value){
 return atof(value.c_str());
}

/*
        convert a string to a double
*/
double str2double(string value){
  return stod(value.c_str());
}


/* CLASS OPTION  ===================================

	An option is defined by a short and a long name
	The - and -- in the option names are optional but recommended and must be specified in the names if wanted

*/
class Option{
protected:
	string var, default_value;
	string short_name;
	string long_name;
	string description;
	bool _is_boolean;

public:
	Option(string short_name, string long_name, string description, string default_value){
		this->short_name = short_name;
		this->long_name = long_name;
        this->description = description;
        this->default_value = default_value;
		var = default_value;
		_is_boolean = false;
	}

	/*
		create a boolean option
	*/
	Option(string short_name, string long_name, string description, bool default_value){
		this->short_name = short_name;
		this->long_name = long_name;
        this->description = description;
        this->default_value = bool2str(default_value);
		var = bool2str(default_value);
		_is_boolean = true;
	}

	~Option(){}
    
    string get_short_name()const{return short_name;}
    string get_long_name()const{return long_name;}
    string get_description()const{return description;}
    string get_default() const{ return default_value;}
    string get_var() const{ return var;}
    void set_var(string& new_var){ var = new_var;}
    void set_var(bool new_val){ var = bool2str(new_val);}
    bool is_boolean(){return _is_boolean;}

};







/* CLASS OPTIONPARSER  ===================================

	This class implements a very easy to use c++ option parser
	Options can be added and then their value accessed with [] operator
	All options values are string, the user must convert them himself

*/
class OptionParser{

protected:
    vector<Option> options;
    int max_size_short; // max size of short names of options, usefull for a beautifull help print
    int max_size_long;  // max size of long names of options, usefull for a beautifull help print
    int max_size_desc;  // max size of desciption of options, usefull for a beautifull help print

    bool has_option(string name){
        for( size_t i=0; i<options.size(); i++ )
            
			if(options[i].get_short_name() == name || options[i].get_long_name() == name){
				// an argument name has been recognised
				return true;				
			}
			// else option_recognised stays to false
		return false;
    }

	// returns a string of n spaces
    std::string repeat_str( const std::string &word, int times ) {
	   std::string result ;
	   result.reserve(times*word.length()); // avoid repeated reallocation
	   for ( int a = 0 ; a < times ; a++ ) 
	      result += word ;
	   return result ;
	}

    // returns max
    int max(int a, int b){
    	return (a<b)? b: a;
    }


public:
	OptionParser(){
		max_size_short = max_size_long = max_size_desc = 0;
	}

	~OptionParser(){
	}

	/*
		Shows help with aligned columns
	*/
	void show_help(){
		string sn = "Short_name";
		string ln = "Long_name";
		string desc = "Description";
		// fisrt calculate the max size of each printed column :
		int max_size_s = max(max_size_short, sn.size());
		int max_size_l = max(max_size_long, ln.size());
		int max_size_d = max(max_size_desc, desc.size());

		// then print nicely the help
		string separator = repeat_str("=", max_size_s + max_size_l + max_size_d + 20);

		cout << endl << "Help :" << endl;
		cout << separator << endl;


		string sn_spaces = repeat_str( " " , max_size_s - sn.size() +2 ); // garanty 2 spaces at least
		string ln_spaces = repeat_str( " " , max_size_l - ln.size() +2 ); // garanty 2 spaces at least
		string desc_spaces = repeat_str( " " , max_size_d - desc.size() +2 ); // garanty 2 spaces at least

		// print the title line
		cout << sn << sn_spaces << ln << ln_spaces << desc << desc_spaces << "Default_value" << endl;
		cout << separator << endl;

		// print the options
		for (vector<Option>::iterator it=options.begin(); it!=options.end(); it++){
			string sn = it->get_short_name();
			string ln = it->get_long_name();
			string desc = it->get_description();

			sn_spaces = repeat_str( " " , max_size_s - sn.size() +2 ); // garanty 2 spaces at least
			ln_spaces = repeat_str( " " , max_size_l - ln.size() +2 ); // garanty 2 spaces at least
			desc_spaces = repeat_str( " " , max_size_d - desc.size() +2 ); // garanty 2 spaces at least

			cout << sn << sn_spaces << ln << ln_spaces << desc << desc_spaces << it->get_default() << endl;
		}

		cout << separator << endl;
		cout << endl;
	}

	void add_option(string short_name, string long_name, string description, string default_value){
		if(has_option(short_name))
			throw string("Error when adding option " + long_name + ". Option already exists !");
		
		Option opt(short_name, long_name, description, default_value);
		options.push_back(opt);
		// update max sizes
		max_size_short = max(max_size_short, short_name.size());
		max_size_long = max(max_size_long, long_name.size());
		max_size_desc = max(max_size_desc, description.size());
	}

	// to add a boolean option
	void add_option(string short_name, string long_name, string description){
		if(has_option(short_name))
			throw string("Error when adding option " + long_name + ". Option already exists !");

		Option opt(short_name, long_name, description, false);
		options.push_back(opt);
		// update max sizes
		max_size_short = max(max_size_short, short_name.size());
		max_size_long = max(max_size_long, long_name.size());
		max_size_desc = max(max_size_desc, description.size());
	}


	/*
		This function changes default option values to those which are read on the command line
	*/
	bool parse_options(int& argc, char**& argv){
		// for each argument, check all options
		for(int i = 1; i<argc; i++){ // begin at 1 because of programm name
			string arg(argv[i]);
			bool option_recognised = false;
			for( vector<Option>::iterator it=options.begin(); it!=options.end(); it++ ){
				if(it->get_short_name() == arg || it->get_long_name() == arg){
					// an argument name has been recognised
					option_recognised = true;

					if( it->is_boolean() ){
						it->set_var(true);
					}
					else{
                        i++; // increase i
						// check if there is an argument after
						if(i==argc || has_option(argv[i]) ){
							cout << "Non boolean option " << arg << " needs an argument !" << endl;
							return false;
						}
                        string var(argv[i]);
						it->set_var(var);
					}
				}
				// else option_recognised stays to false
			}
			// if option is not recognised
			if(!option_recognised){
				cout << "Unknown option " << arg << endl;
				show_help();
				return false;
			}
		}
		return true;
	}


	/*
		Get the option values as string
	*/
	string operator[](string name) const{
		bool option_recognised = false;
		string var;

        for( size_t i=0; i<options.size(); i++ )
            
			if(options[i].get_short_name() == name || options[i].get_long_name() == name){
				// an argument name has been recognised
				option_recognised = true;
				var = options[i].get_var();
			}
			// else option_recognised stays to false
	
		// if option is not recognised
		if(!option_recognised){
			throw string( "Unknown option " + name );
		}
		
		return var;
	}


};//end of class


}; //end of cmdp namespace

#endif
