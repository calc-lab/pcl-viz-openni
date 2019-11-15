#ifndef PCAPPARSER_H
#define PCAPPARSER_H

#include <queue>
#include <fstream>
#include <iostream>
#include <stdint.h>
using namespace std;

#define  PCAP_FILE_MAGIC_1   0Xd4
#define  PCAP_FILE_MAGIC_2   0Xc3
#define  PCAP_FILE_MAGIC_3   0Xb2
#define  PCAP_FILE_MAGIC_4   0Xa1


/*pcap file header*/
struct PcapFileHeader
{
    uint8_t   magic[4];
    uint16_t   version_major;
    uint16_t   version_minor;
    int32_t    thiszone;      /*时区修正*/
    uint32_t   sigfigs;       /*精确时间戳*/
    uint32_t   snaplen;       /*抓包最大长度*/
    uint32_t   linktype;      /*链路类型*/
};



/*pcap packet header*/
struct PcapPktHeader
{
    uint32_t   seconds;     /*秒数*/
    uint32_t   u_seconds;   /*毫秒数*/
    uint32_t   caplen;      /*数据包长度*/
    uint32_t   len;         /*文件数据包长度*/
};

class PcapParser
{
public:
    PcapParser(std::string pcapfilename);
    PcapParser();

    bool loadPcapData(std::string pcapfilename);

    bool isFileOpened();

    /**
     * @brief getStreamData
     * @param data
     * @param inlen
     * @param header
     * @return inlen必须和PcapPktHeader中获取的大小一致才返回true
     */
    bool getStreamData(char*data, int inlen, PcapPktHeader &header);

    std::ifstream & getFileHeader();

private:
    bool openPcap(std::string pcapfilename);
    bool is_open;
    std::ifstream  fileHandler;
};

#endif // PCAPPARSER_H
