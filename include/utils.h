#ifndef UTIL_H
#define UTIL_H

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
void read_csv_path(std::string path, vector<double>& path_x, vector<double>& path_y, vector<double>& path_goal){
  //loading path for debug
  int line_counter = 0;
  std::fstream fin;     
  fin.open(path, std::ios::in);

  std::string line, word, temp;

  while (fin >> temp) { 
    // read an entire row and 
    // store it in a string variable 'line' 
    std::getline(fin, line);   
    // used for breaking words 
    std::stringstream s(line); 
    // read every column data of a row and 
    // store it in a string variable, 'word' 
    while (getline(s, word, ',')) {  
      // add all the column data 
      // of a row to a vector
      std::cout<<word<<std::endl;
      if (line_counter==0){
        path_x.push_back(std::stod(word)); 
      }
      else{
        path_y.push_back(std::stod(word)); 
      }
      
    }	
    line_counter++;
  }
  std::cout<<"Read Path:"<<std::endl;

  for (auto i=0; i<path_y.size(); i++){
    std::cout<<path_x.at(i)<<","<< path_y.at(i)<<std::endl;
  }

  path_goal.push_back(path_x.back());
  path_goal.push_back(path_y.back());
}

#endif /* UTIL_H */
