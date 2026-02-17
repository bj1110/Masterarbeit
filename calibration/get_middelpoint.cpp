#include <iostream>
#include <filesystem>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>

std::vector<double> get_values(const std::filesystem::path& path, size_t axis_row, int posfilter=9, bool at_end=true);
double mean(const std::vector<double>& values);
double std_dev(const std::vector<double>& values, double mean);

enum Agent{agent1 =1, agent2=2};
enum Row {X_ROW= 2, Y_ROW=1}; 

struct value{
    double mean_;
    double std_dev_;
    int agent_;
    int pos_; 
    int row_; 
    
    friend std::ostream&  operator<<( std::ostream& os, const value& v){
        char c;
        if(v.row_ == 2){
            c='X';
        }
        else if(v.row_ == 1){
            c='Y';
        } else{
            c='F'; 
        }

        os << "agent" << v.agent_ <<" "<<c << " @ "<<v.pos_ <<": "<< v.mean_ <<" " <<v.std_dev_;
        return os; 
    }

    value(const std::filesystem::path& p, int agent, int pos, int row)
    : pos_(pos), agent_(agent), row_(row)
    {
        bool at_end=true;
        if( (agent==1 &&( pos==1 || pos == 3)) || (agent==2 &&(pos==5 || pos==7))){
            at_end=false; 
        }
        std::vector<double> v = get_values(p, row , pos_ , at_end); 
        mean_ = mean(v);
        std_dev_=std_dev(v, mean_);  
    }
}; 



int main(int argc, char** argv){
    const char* home = std::getenv("HOME");
    if(!home){
        std::cerr << "HOME envirement varibale not set" <<std::endl;
        return -1;
    }

    std::filesystem::path baselines_dir (std::filesystem::path(home)/"Projects/Masterarbeit/ros2_ws/src/ik_processing/data/dset/baselines");
    std::filesystem::path agent1_dir = baselines_dir / "agent1";
    std::filesystem::path agent2_dir = baselines_dir / "agent2";

    value a1_9_x {agent1_dir, agent1, 9, X_ROW};
    value a1_9_y {agent1_dir, agent1, 9, Y_ROW};
    value a2_9_x {agent2_dir, agent2, 9, X_ROW};
    value a2_9_y {agent2_dir, agent2, 9, Y_ROW};

    value a1_1_x {agent1_dir, agent1, 1, X_ROW};
    value a1_1_y {agent1_dir, agent1, 1, Y_ROW};
    value a2_1_x {agent2_dir, agent2, 1, X_ROW};
    value a2_1_y {agent2_dir, agent2, 1, Y_ROW};

    std::cout<<a1_9_x<<std::endl; 
    std::cout<<a1_9_y<<std::endl; 
    std::cout<<a2_9_x<<std::endl; 
    std::cout<<a2_9_y<<std::endl; 

    std::cout << "--------" <<std::endl; 
    
    std::cout<<a1_1_x<<std::endl; 
    std::cout<<a1_1_y<<std::endl; 
    std::cout<<a2_1_x<<std::endl; 
    std::cout<<a2_1_y<<std::endl; 

    return 0;    
}

std::vector<double> get_values(const std::filesystem::path& path, size_t axis_row, int posfilter, bool at_end){
    std::vector<double> values;
    std::string filter = std::to_string(posfilter);
    int i=0; 
    for(const auto& movement_dir: std::filesystem::directory_iterator(path)){
        std::string movement_name = movement_dir.path().filename().string();
        if(movement_name.find(filter) == std::string::npos){
            continue; 
        }
        for(const auto& file: std::filesystem::recursive_directory_iterator(movement_dir)){
            if(!file.is_regular_file()){
                continue;
            }
            std::ifstream inputfile {file.path()};
            if(!inputfile.is_open()){
                std::cerr << "file could not be opened"<< std::endl;
                return {};
            }
            std::vector<std::vector<double>> rows;
            std::string line;
            while(std::getline(inputfile, line)){
                std::istringstream iss {line};
                std::vector<double> row;
                double f;
                while( iss >> f){
                    row.push_back(f);
                }
                if(!row.empty()){
                    rows.push_back(row);
                }
            }
            if(at_end){
                values.push_back(rows[axis_row].back());
            }
            else{
                values.push_back(rows[axis_row].front()); 
            }
        }
    }
    return values;  
}
    
double mean(const std::vector<double>& values){
    double sum = 0;
    double num_elements = values.size();
    for(const auto& v: values){
        sum+=v;
    }
    return sum/num_elements; 
}

double std_dev(const std::vector<double>& values, double mean){
    double num_elements = values.size();
    double sum=0;
    for(const auto& v: values){
        sum += (v-mean)*(v-mean);
    }
    double res = sum / num_elements;
    return std::sqrt(res);
}
           