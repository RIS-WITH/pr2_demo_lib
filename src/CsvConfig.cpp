#include "pr2_demo_lib/CsvConfig.h"

#include <iostream>

std::string getFileAsString(std::ifstream& file)
{
  std::string file_str;
  std::string line;
  while(file)
  {
    line = "";
    std::getline(file, line);
    file_str += line + "\n";
  }

  return file_str;
}

std::vector<std::vector<std::string>> getCells(const std::string& str)
{
  std::vector<std::vector<std::string>> res;
  std::vector<std::string> lines;
  std::string segment;
  std::string cell;

  std::istringstream istream_str(str);

  while(std::getline(istream_str, segment, '\n'))
  {
    res.push_back({});
    std::istringstream istream_line(segment);
    while(std::getline(istream_line, cell, ','))
      res.back().push_back(cell);
  }

  return res;
}

std::vector<std::vector<trial_t>> readConfigFiles(const std::string& digit_path, const std::string& incongruance_path)
{
  std::vector<std::vector<trial_t>> res;

  std::ifstream digit_file(digit_path);
  std::ifstream incongruance_file(incongruance_path);

  if(digit_file.is_open() && incongruance_file.is_open())
  {
    std::string digit_file_str = getFileAsString(digit_file);
    std::string incongruance_file_str = getFileAsString(incongruance_file);

    std::vector<std::vector<std::string>> digit_cells = getCells(digit_file_str);
    std::vector<std::vector<std::string>> incongruance_cells = getCells(incongruance_file_str);

    for(int participant = 1; participant < digit_cells.size(); participant++)
    {
      res.push_back({});
      for(int trial = 1; trial < digit_cells[participant].size(); trial++)
      {
        trial_t trial_data;
        trial_data.digit = digit_cells[participant][trial];
        trial_data.incongruance = (incongruance_cells[participant][trial] == "1");
        res.back().push_back(trial_data);
      }
    }

    digit_file.close();
    incongruance_file.close();
  }
  else
    std::cout << "bad files path" << std::endl;

  return res;
}

std::vector<std::vector<std::string>> readQuestionFile(const std::string& path)
{
  std::ifstream file(path);
  if(file.is_open())
  {
    std::string file_str = getFileAsString(file);
    return getCells(file_str);
  }
  else
  {
    std::cout << "Bad Question file" << std::endl;
    return {};
  }
}

void writeCsvFile(const std::vector<std::vector<std::string>>& data, const std::string& path)
{
  std::ofstream file(path);
  if(file.is_open())
  {
    bool first_row = true;
    for(auto& row : data)
    {
      if(!first_row)
        file << std::endl;
      else
        first_row = false;

      for(auto& cell : row)
        file << cell << ",";
    }
  }
  else
    std::cout << "Bad Question file" << std::endl;
}