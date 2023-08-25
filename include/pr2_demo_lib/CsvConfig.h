#ifndef PR2_DEMO_LIB_CSVCONFIG_H
#define PR2_DEMO_LIB_CSVCONFIG_H

#include <string>
#include <fstream>
#include <sstream>
#include <vector>

struct trial_t
{
  std::string digit;
  bool incongruance;

  std::string toString()
  {
    return "digit = " + digit + " : incongruance = " + (incongruance ? "true" : "false");
  }
};

std::string getFileAsString(std::ifstream& file);

std::vector<std::vector<std::string>> getCells(const std::string& str);

std::vector<std::vector<trial_t>> readConfigFiles(const std::string& digit_path, const std::string& incongruance_path);
std::vector<std::vector<std::string>> readQuestionFile(const std::string& path);

void writeCsvFile(const std::vector<std::vector<std::string>>& data, const std::string& path);

#endif // PR2_DEMO_LIB_CSVCONFIG_H