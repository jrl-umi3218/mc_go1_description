#include <iostream>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string/classification.hpp> // is_any_of
#include <boost/algorithm/string/split.hpp>

/* ---------------------------------------------------------------------------- */
/* If the unit system of the mesh model specified by the argument is not 1.0,   */
/* the coordinate value of qcFile will be scaled and overwritten by that value. */
/* If it is 1.0, it does nothing.                                               */
/* ---------------------------------------------------------------------------- */
int main(int argc, char ** argv)
{
  using namespace boost::property_tree;

  if(argc != 3)
  {
    std::cout << "Usage: scaling_xyz daeFile qcFile" << std::endl;
    return 1;
  }

  std::string dae_file = argv[1];
  std::string qc_file = argv[2];
  double scale = 1.0;

  // Read the mesh model and check the unit system
  ptree pt;
  read_xml(dae_file, pt);

  if(boost::optional<double> meter = pt.get_optional<double>("COLLADA.asset.unit.<xmlattr>.meter"))
  {
    scale = meter.get();
    // std::cout << scale << std::endl;
  }

  // When the unit system of the mesh model is not 1.0
  if(scale < 1.0 && scale > 0.0)
  {
    // Open qcFile
    std::ifstream ifile(qc_file);
    if(!ifile.is_open())
    {
      std::cout << qc_file + " cannot be opened." << std::endl;
      return 1;
    }

    // Open the working file of gcFile
    std::ofstream ofile(qc_file + ".scale");
    if(!ofile.is_open())
    {
      std::cout << qc_file + ".scale cannot be opened." << std::endl;
      return 1;
    }

    // Convert line by line
    std::string line;
    std::vector<std::string> xyz;
    while(std::getline(ifile, line))
    {
      boost::algorithm::split(xyz, line, boost::is_any_of(" "));
      if(xyz.size() == 3)
      {
        double x = stod(xyz[0]) * scale;
        double y = stod(xyz[1]) * scale;
        double z = stod(xyz[2]) * scale;
        ofile << std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) << std::endl;
      }
      else
      {
        // Not a coordinate value
        ofile << line << std::endl;
      }
      xyz.clear();
    }

    ifile.close();
    ofile.close();

    // Overwrite qcFile
    rename((qc_file + ".scale").c_str(), qc_file.c_str());
  }

  return 0;
}

