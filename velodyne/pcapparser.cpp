#include "pcapparser.h"
#include <string.h>
PcapParser::PcapParser(string pcapfilename)
{
    openPcap(pcapfilename);
}

PcapParser::PcapParser()
{
    is_open = false;
}

bool PcapParser::loadPcapData(string pcapfilename)
{
    return openPcap(pcapfilename);
}

bool PcapParser::isFileOpened()
{
    return is_open;
}

bool PcapParser::getStreamData(char *data, int inlen, PcapPktHeader &header)
{
    if (!is_open || !data)
        return 0;
    PcapPktHeader  packetHeader = {0};
    while(!fileHandler.eof())
    {
        memset(&packetHeader, 0, sizeof(packetHeader));
        fileHandler.read((char*)&packetHeader, sizeof(packetHeader));
        if (packetHeader.len > 0)
            fileHandler.read((char*)data, packetHeader.len);
        else
            return 0;

        header = packetHeader;
        if (inlen != header.len){
            continue;
        }
        else{
            return 1;
        }
    }
    return 0;
}



ifstream &PcapParser::getFileHeader()
{
    if (is_open)
        return fileHandler;
}

bool PcapParser::openPcap(string pcapfilename)
{
    PcapFileHeader  pcapFileHeader = {0};

    is_open = true;
    fileHandler.open(pcapfilename);
    if (!fileHandler)
    {
        cout << "The file does not exits or file name is error" << endl;
        is_open = false;
        return is_open;
    }

    //读取pcap文件头部长度
    fileHandler.read((char*)&pcapFileHeader, sizeof(pcapFileHeader));
    if (pcapFileHeader.magic[0] != PCAP_FILE_MAGIC_1 || pcapFileHeader.magic[1] != PCAP_FILE_MAGIC_2 ||
            pcapFileHeader.magic[2] != PCAP_FILE_MAGIC_3 || pcapFileHeader.magic[3] != PCAP_FILE_MAGIC_4)
    {
        cout << "The file is not a pcap file" << endl;
        is_open = false;
        return is_open;
    }
    return is_open;
}
