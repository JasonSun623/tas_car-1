#include <iostream>
#include <map>
#include <termios.h>

#include "wifi_localization/FFNN.h"
#include "ros/ros.h"

#include "wifi_scan/Fingerprint.h"

using namespace std;

#define rssi_zero -100.0
#define network_MSE 0.0002

/*
This node is intended to localize the vehicle in specific positions
It uses the signal strengths of all the detected wifi signals
When you run the program for the first time you have to record samples when the vehicle is in the wanted position
The positions are set hardcoded for now
When you have recorded the samples and trained the network with them you can save the data to load another time
The localization worked on every laptop I tried but on the car the wifi_scan node had some problems getting 
current data and always published the same message without any update

Since it didn't work on the car I stopped developing it further and the following points are missing:
- final decision in which position the car is from the FFNN output
- sending the pose estimate to the amcl localization node
- expanding the training data by adding the scans partially (for example deleting single wifi signals) 
	would improve detection chance when some wifi access points went down between the sample recording and the localization
*/

struct wifiSignal{
    string name;
    uint id;
    double rssi;
};

struct wifiSample{
    uint signalcount;
    vector<wifiSignal*> wifi_signals;
};

struct Coordinates{
    double x;
    double y;
    double z;
    double alpha;
};

struct Location{
    string name;
    uint id;
    struct Coordinates;
    uint samplecount = 0;
    vector<wifiSample*> wifi_samples;
};

uint hold;

ros::Subscriber wifi_sub;

//mapping from wifi name (mac-adress string) to ID
map<string, uint> mapping;

//Highest assigned ID for wifi
uint max_id = 0;

//Assign new IDs to new detected wifi
bool accept_new_wifi = true;


uint Number_of_positions;
vector<Location*> position_data;


vector<wifiSignal*> all_detected_wifi, current_detected_wifi;

FFNN_network* neural_network;

void wifi_callback(const wifi_scan::Fingerprint& msg)
{
	//Clear list of currently detected wifi signals
    current_detected_wifi.clear();

    uint ID;
    wifiSignal* temp_signal;

	//Go through the elements of the wifi signal msg
    for(vector<wifi_scan::AddressRSSI>::const_iterator signal = msg.list.begin(); signal != msg.list.end(); ++signal)
    {
		//Get ID from mapping
        ID = mapping[(*signal).address];
		
		//When no ID is found and ID assigning is enabled -> assign new ID
        if(!ID && accept_new_wifi)
        {
            max_id++;
            mapping[(*signal).address] = max_id;
            ID = max_id;
            //cout << "added " << msg.list.at(i).address << "as id: " << max_id << endl;

            temp_signal = new wifiSignal;
            temp_signal->name = (*signal).address;
            temp_signal->id = max_id;

			//Push wifi signal in the list of all the ever detected signals
            all_detected_wifi.push_back(temp_signal);
        }

		//When wifi has an ID -> push it into the currently detected wifi list
        if(ID)
        {
            temp_signal = all_detected_wifi.at(ID - 1);
            temp_signal->rssi = (*signal).rssi;

            current_detected_wifi.push_back(temp_signal);
        }

    }
}

//Non blocking getchar function
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

//Records a wifi sample at the given position and pushes it into the sample list
void record_sample(Location* position)
{
    wifiSample* temp_sample = new wifiSample;
    wifiSignal* temp_signal;

    for(vector<wifiSignal*>::iterator curr_wifi = current_detected_wifi.begin(); curr_wifi != current_detected_wifi.end(); ++curr_wifi)
    {
        temp_signal = new wifiSignal;

        //temp_signal->name = (*curr_wifi)->name; //not necessary for the training data

        temp_signal->id = (*curr_wifi)->id;
        temp_signal->rssi = (*curr_wifi)->rssi;

        temp_sample->wifi_signals.push_back(temp_signal);
        temp_sample->signalcount++;
    }

    position->wifi_samples.push_back(temp_sample);
    position->samplecount++;
}

//Saves 4 wifi samples with 2.5s pause in between
void record_data(Location* position)
{
    system("clear");
    cout << "Recording some data at this position" << endl;
    ros::Rate Hz10(10);

    uint counter = 0;

    while(counter <= 100)
    {
        ros::spinOnce();

        switch(counter)
        {
        case 25:
            cout << "1/4" << endl;
            record_sample(position);
            break;

        case 50:
            cout << "2/4" << endl;
            record_sample(position);
            break;

        case 75:
            cout << "3/4" << endl;
            record_sample(position);
            break;

        case 100:
            cout << "4/4" << endl;
            record_sample(position);
            break;

        }

        Hz10.sleep();
        counter++;
    }
}

//Generate the training data for the neural network by going through all the positions
// and iterating through all the samples.
// Inputs are the signal strengths (Inputnumber == wifi-ID)
// Outputs are the pseudo probabilities of beeing at the position (Outputnumber == position-ID)
// Outputs are independent -> not summing up to 1
void generate_training_data()
{
    Training_set* new_training_set;

    double* inputs = (double*) malloc(max_id * sizeof(double));
    double* outputs = (double*) malloc(Number_of_positions * sizeof(double));

    for(vector<Location*>::iterator loc = position_data.begin(); loc != position_data.end(); ++loc)
    {
        for(vector<wifiSample*>::iterator sample = (*loc)->wifi_samples.begin(); sample != (*loc)->wifi_samples.end(); ++sample)
        {
            for(uint i = 0; i < max_id; i++)
                inputs[i] = rssi_zero;

            for(uint i = 0; i < Number_of_positions; i++)
                outputs[i] = 0;

            for(vector<wifiSignal*>::iterator signal = (*sample)->wifi_signals.begin(); signal != (*sample)->wifi_signals.end(); ++signal)
            {
                inputs[(*signal)->id - 1] = (*signal)->rssi;
            }

            outputs[(*loc)->id] = 1;

            new_training_set = new Training_set(inputs, max_id, outputs, Number_of_positions);
            neural_network->add_training_set(new_training_set);
        }
    }

    neural_network->normalize_training_set();
}

//Load data from file
bool load_data()
{
    cout << "Enter path to load data from (without extensions)" << endl;
    string base_path, net_path, wifi_path;
    cin >> base_path;

    net_path = base_path + ".netdata";
    wifi_path = base_path + ".wifidata";

    try
    {
        if(!neural_network)
            neural_network = new FFNN_network(0,0,0,network_MSE);

        if(!neural_network->load(net_path.c_str()))
            return false;

        FILE* infile = fopen(wifi_path.c_str(), "r");
        if(!infile)
        {
            cout << "Error opening file " << wifi_path << endl;
            return false;
        }

        uint id;
        char name[80];
        wifiSignal* temp_signal;

        all_detected_wifi.clear();

        mapping.clear();
        max_id = 0;

        while(!feof(infile))
        {
            if(!fscanf(infile, "%d#%s\n", &id, name))
            {
                fclose(infile);
                break;
            }
            else
            {
                temp_signal = new wifiSignal;

                mapping[name] = id;
                temp_signal->id = id;
                temp_signal->name = name;
                max_id++;

                all_detected_wifi.push_back(temp_signal);
            }
        }

        fclose(infile);

        return true;
    }
    catch(exception e)
    {
        cout << "Error loading" << endl;
        return false;
    }
}

//Save data to file
bool save_data(const char* base_path)
{
    string net_path(base_path), wifi_path(base_path);
    
    net_path = net_path + ".netdata";
    wifi_path = wifi_path + ".wifidata";

    try
    {
        neural_network->save(net_path.c_str());

        FILE* outfile = fopen(wifi_path.c_str(), "w");

        for(vector<wifiSignal*>::iterator signal = all_detected_wifi.begin(); signal != all_detected_wifi.end(); ++signal)
        {
            fprintf(outfile, "%d#%s\n", (*signal)->id, (*signal)->name.c_str());
        }

        fclose(outfile);
		
		return true;
    }
    catch(exception e)
    {
        cout << "Error saving" << endl;
        return false;
    }


}

//ask for path and save data
bool save_data_ask()
{
    cout << "Enter path to save data" << endl;
    string base_path;
    cin >> base_path;

	return save_data(base_path.c_str());
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "wifi_locator");
    ros::NodeHandle nh;
    ros::Rate Hz10(10);

    ros::Duration S5(5.0);

    Number_of_positions = 8;

    Location* temp_loc;

    temp_loc = new Location;
    temp_loc->id = 0;
    temp_loc->name = "Position 0 (first corner)";
    position_data.push_back(temp_loc);

    temp_loc = new Location;
    temp_loc->id = 1;
    temp_loc->name = "Position 1 (between first and second corner)";
    position_data.push_back(temp_loc);

    temp_loc = new Location;
    temp_loc->id = 2;
    temp_loc->name = "Position 2 (second corner)";
    position_data.push_back(temp_loc);

    temp_loc = new Location;
    temp_loc->id = 3;
    temp_loc->name = "Position 3 (between second and third corner)";
    position_data.push_back(temp_loc);

    temp_loc = new Location;
    temp_loc->id = 4;
    temp_loc->name = "Position 4 (third corner)";
    position_data.push_back(temp_loc);

    temp_loc = new Location;
    temp_loc->id = 5;
    temp_loc->name = "Position 5 (between third and fourth corner)";
    position_data.push_back(temp_loc);

    temp_loc = new Location;
    temp_loc->id = 6;
    temp_loc->name = "Position 6 (fourth corner)";
    position_data.push_back(temp_loc);

    temp_loc = new Location;
    temp_loc->id = 7;
    temp_loc->name = "Position 7 (between fourth and first corner)";
    position_data.push_back(temp_loc);

    wifi_sub = nh.subscribe("/wifi_fp", 1, &wifi_callback);


    uint selection = 1;
    uint pos_selection = 0;

    while(1)
    {
        char c = getch();

        if(c == '1')
        {
            selection = 1;
        }
        else if(c == '2')
        {
            selection = 2;
        }
        else if(c == '3')
        {
            selection = 3;
        }
        else if(c == '4')
        {
            selection = 4;
        }
        else if(c == 'w')
        {
            if(selection == 3)
            {
                if(pos_selection == 0)
                    pos_selection = Number_of_positions - 1;
                else
                    pos_selection--;
            }
        }
        else if(c == 's')
        {
            if(selection == 3)
            {
                pos_selection++;
                if(pos_selection > Number_of_positions - 1)
                    pos_selection = 0;
            }
            if(selection == 10)
            {
                save_data_ask();
            }
        }
        else if(c == 'l')
        {
            if(load_data())
            {
                selection = 10;
                accept_new_wifi = false;
            }
            else
            {
                cout << "Loading failed" << endl;
                S5.sleep();
            }
        }
        else if(c == 'r')
        {
            if(selection == 3)
                record_sample(position_data.at(pos_selection));
        }
        else if(c == 't')
        {
            if(selection == 3)
                record_data(position_data.at(pos_selection));
        }
        else if(c == 'q')
        {
            cout << "Bye :)"<< endl;
            return 0;
        }

        Hz10.sleep();
        ros::spinOnce();

        system("clear");


        if(selection == 1)
        {
            cout << "<1> All detected networks" << endl;
            cout << "(2) Current detected networks" << endl;
            cout << "(3) Collect new training data" << endl;
            cout << "(4) Start training and launch locator" << endl;
            cout << "(l) Load data from another run" << endl;
            cout << "(q) Quit" << endl;
            for(vector<wifiSignal*>::iterator detected = all_detected_wifi.begin(); detected != all_detected_wifi.end(); ++detected)
            {
               cout << "ID: " << (*detected)->id << " address: "  << (*detected)->name << endl;
            }
        }
        else if(selection == 2)
        {
            cout << "(1) All detected networks" << endl;
            cout << "<2> Current detected networks" << endl;
            cout << "(3) Collect new training data" << endl;
            cout << "(4) Start training and launch locator" << endl;
            cout << "(l) Load data from another run" << endl;
            cout << "(q) Quit" << endl;
            for(vector<wifiSignal*>::iterator detected = current_detected_wifi.begin(); detected != current_detected_wifi.end(); ++detected)
            {
               cout << "ID: " << (*detected)->id << " address: "  << (*detected)->name << " rssi: "  << (*detected)->rssi << endl;
            }
        }
        else if(selection == 3)
        {
            cout << "(1) All detected networks" << endl;
            cout << "(2) Current detected networks" << endl;
            cout << "<3> Collect new training data" << endl;
            cout << "(4) Start training and launch locator" << endl;
            cout << "(l) Load data from another run" << endl;
            cout << "(q) Quit" << endl;
            cout << endl;

            cout << "Use 'w' and 's' to switch between the positions" << endl;
            cout << "Press 'r' to record 1 sample  for the selected position" << endl;
            cout << "Press 't' to record 4 samples for the selected position" << endl;
            cout << endl;

            Location* loc = position_data.at(pos_selection);

            cout << "<" << pos_selection << "> " <<  loc->name << " (" << loc->samplecount << " samples)" << endl;

        }
        else if(selection == 4)
        {
            accept_new_wifi = false;

			//create neural network with so many inputs like detected wifi and outputs like positions
            neural_network = new FFNN_network(max_id, 30, Number_of_positions, network_MSE);

            cout << "Generating Training data ..." << endl;
            generate_training_data();

			//Train neural network with a timeout of 10 minutes
            cout << "Training Neural Network ..." << endl;
            neural_network->train(600);

            //cout << "Saving Data ..." << endl;
            //save_data("/home/wolfi/bla/LastWifiNetwork");

            selection = 10;
        }
        else if(selection == 10)
        {
              cout << "(l) Load data from another run" << endl;
              cout << "(s) Save current data for another time" << endl;
              cout << "(q) Quit" << endl;
              cout << endl;

              double* inputs = (double*) malloc(max_id*sizeof(double));
              double* outputs = (double*) malloc(Number_of_positions*sizeof(double));

              for(uint i = 0; i < max_id; i++)
                  inputs[i] = rssi_zero;

              for(vector<wifiSignal*>::iterator signal = current_detected_wifi.begin(); signal != current_detected_wifi.end(); ++signal)
              {
                  inputs[(*signal)->id - 1] = (*signal)->rssi;
              }

              neural_network->request_output(inputs, max_id, outputs, Number_of_positions);

              for(vector<Location*>::iterator loc = position_data.begin(); loc != position_data.end(); ++loc)
                  cout << "<" << (*loc)->id << "> " <<  (*loc)->name << " -> " << outputs[(*loc)->id] << endl;


              free(inputs);
              free(outputs);
        }

    }

    ros::spin();


    return 0;
}
