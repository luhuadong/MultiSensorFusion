#include <boost/function.hpp>
#include <boost/thread.hpp>
#include "pcap.h"
#include <string>

using namespace std;

class PcapReader {
public:
  PcapReader(std::string path, std::string frame_id);
  ~PcapReader();

  void start(boost::function<void(const uint8_t*, const int, double timestamp)> callback);
  void stop();

private:
  bool          loop;
  boost::thread *parse_thr_;
  std::string   pcapPath;
  std::string   m_sFrameId;
  boost::function<void(const uint8_t*, const int, double timestamp)> callback; 
  std::map<std::string, std::pair<int,int>> m_timeIndexMap;
  int m_iTsIndex;
  int m_iUTCIndex;

  void parsePcap();
  void initTimeIndexMap();
};
