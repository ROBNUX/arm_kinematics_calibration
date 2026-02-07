#include "robnux_utilities/utility.hpp"
#include <fstream>
#include "robnux_utilities/loadtxt.hpp"
#include "simple_motion_logger/Logger.h"

using namespace ROBNUXLogging;

namespace kinematics_lib {
RPE_Utility::RPE_Utility() {

}
     
bool  RPE_Utility::ReadCSVFile(const std::string fileName,
                               Eigen::MatrixXd *file_data) {
   std::ostringstream strs;
   if (!file_data) {
      strs.str("");
      strs << " Input data pointer is NULL, stop here!" << std::endl;
      LOG_ERROR(strs);
      return false;
   }
   try {
      auto data = loadtxt(fileName).comments("#")();
      if (data.empty()) {
         strs.str("");
         strs << fileName  << " is empty, stop here!" << std::endl;
         LOG_ERROR(strs);
         return false;
      }
      strs.str("");
      size_t numRows = data.size();
      size_t numCols = data[0].size();
      strs << "data size =" << data.size() <<
      ", each col size=" << data[0].size() << std::endl;
      LOG_INFO(strs);
   
      //strs.str("");
      file_data->resize(numRows, numCols);
      for (size_t i=0; i< numRows; i++) {
         for (size_t j=0; j < numCols; j++) {
            (*file_data)(i, j) = data[i][j];
         }
      }
   } catch(...) {
      strs.str("");
      strs << "File " << fileName << " seems not exist!" << std::endl;
      LOG_ERROR(strs);
   }
   return true;
}
bool  RPE_Utility::WriteCSVFile(const std::string fileName,
                                const Eigen::MatrixXd &data) {
   std::ofstream fout(fileName);
   size_t numRows = data.rows();
   size_t numCols = data.cols();
   for(size_t i=0; i < numCols; i++) {
      for(size_t j=0; j < numRows; j++) {
            fout << data(j, i) << ' ';
      }
      fout << endl;
   }
   fout.close();
   return true;
}
}