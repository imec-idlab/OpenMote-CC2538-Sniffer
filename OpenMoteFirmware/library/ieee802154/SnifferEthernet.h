/**
 * @file       SnifferEthernet.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SNIFFER_ETHERNET_H_
#define SNIFFER_ETHERNET_H_

#include "SnifferCommon.h"
#include "Ethernet.h"

class SnifferEthernet : public SnifferCommon
{
public:
    SnifferEthernet(Board& board, Radio& radio, Ethernet& ethernet);
    void init(void);
    void processRadioFrame(void);
private:
    Ethernet ethernet_;
};

#endif /* SNIFFER_ETHERNET_H_ */
