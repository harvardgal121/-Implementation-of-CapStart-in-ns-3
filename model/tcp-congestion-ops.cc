/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015 Natale Patriciello <natale.patriciello@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
 
#include "tcp-congestion-ops.h"
#include "tcp-socket-base.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("TcpCongestionOps");

NS_OBJECT_ENSURE_REGISTERED (TcpCongestionOps);

TypeId
TcpCongestionOps::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpCongestionOps")
    .SetParent<Object> ()
    .SetGroupName ("Internet")
  ;
  return tid;
}

TcpCongestionOps::TcpCongestionOps () : Object ()
{
}

TcpCongestionOps::TcpCongestionOps (const TcpCongestionOps &other) : Object (other)
{
}

TcpCongestionOps::~TcpCongestionOps ()
{
}


// RENO

NS_OBJECT_ENSURE_REGISTERED (TcpNewReno);

TypeId
TcpNewReno::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpNewReno")
    .SetParent<TcpCongestionOps> ()
    .SetGroupName ("Internet")
    .AddConstructor<TcpNewReno> ()
    .AddAttribute ("DataRate", 
                   "The sender bandwidth",
                   DataRateValue (DataRate ("5Mbps")),
                   MakeDataRateAccessor (&TcpNewReno::m_senderBandwidth),
                   MakeDataRateChecker ())
  ;
  return tid;
}

TcpNewReno::TcpNewReno (void) : TcpCongestionOps (),
	m_cWndInSegments (0),
	m_firstPacketInPair (true),
	m_maxSsThresh(100),
	m_firstSample (true),
	m_isStart(true)
	
	
{
  NS_LOG_FUNCTION (this);
}

TcpNewReno::TcpNewReno (const TcpNewReno& sock)
  : TcpCongestionOps (sock),
  m_maxSsThresh (sock.m_maxSsThresh)

{
  NS_LOG_FUNCTION (this);
}

TcpNewReno::~TcpNewReno (void)
{
}

/**
 * \brief Tcp NewReno slow start algorithm
 *
 * Defined in RFC 5681 as
 *
 * > During slow start, a TCP increments cwnd by at most SMSS bytes for
 * > each ACK received that cumulatively acknowledges new data.  Slow
 * > start ends when cwnd exceeds ssthresh (or, optionally, when it
 * > reaches it, as noted above) or when congestion is observed.  While
 * > traditionally TCP implementations have increased cwnd by precisely
 * > SMSS bytes upon receipt of an ACK covering new data, we RECOMMEND
 * > that TCP implementations increase cwnd, per:
 * >
 * >    cwnd += min (N, SMSS)                      (2)
 * >
 * > where N is the number of previously unacknowledged bytes acknowledged
 * > in the incoming ACK.
 *
 * The ns-3 implementation respect the RFC definition. Linux does something
 * different:
 * \verbatim
u32 tcp_slow_start(struct tcp_sock *tp, u32 acked)
  {
    u32 cwnd = tp->snd_cwnd + acked;

    if (cwnd > tp->snd_ssthresh)
      cwnd = tp->snd_ssthresh + 1;
    acked -= cwnd - tp->snd_cwnd;
    tp->snd_cwnd = min(cwnd, tp->snd_cwnd_clamp);

    return acked;
  }
  \endverbatim
 *
 * As stated, we want to avoid the case when a cumulative ACK increases cWnd more
 * than a segment size, but we keep count of how many segments we have ignored,
 * and return them.
 *
 * \param tcb internal congestion state
 * \param segmentsAcked count of segments acked
 * \return the number of segments not considered for increasing the cWnd
 */
uint32_t
TcpNewReno::SlowStart (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);

  if (segmentsAcked >= 1)
    {
      tcb->m_cWnd += tcb->m_segmentSize;
      NS_LOG_INFO ("In SlowStart, updated to cwnd " << tcb->m_cWnd << " ssthresh " << tcb->m_ssThresh);
      return segmentsAcked - 1;
    }

  return 0;
}

uint32_t
TcpNewReno::LimitedSlowStart (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);

 if (segmentsAcked >= 1)
    {
      int k;
      k = (int)(tcb->m_cWnd/(double)(0.5*m_maxSsThresh*tcb->m_segmentSize));
      NS_LOG_INFO ("k = " << k);
      tcb->m_cWnd += (int)(tcb->m_segmentSize/k);
      NS_LOG_INFO ("In LimitedSlowStart, updated to cwnd " << (int)(tcb->m_cWnd/tcb->m_segmentSize) << " ssthresh " << (int)(tcb->m_ssThresh/tcb->m_segmentSize));
      return segmentsAcked - 1;
	}

  return 0;
}

uint32_t
TcpNewReno::CapStart (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);

  if (segmentsAcked >= 1)
    {
      if (((tcb->m_cWnd <= m_maxSsThresh*tcb->m_segmentSize) && (tcb->m_cWnd < tcb->m_ssThresh))|| ((tcb->m_ssThresh < m_maxSsThresh*tcb->m_segmentSize)&& (tcb->m_cWnd <= m_maxSsThresh*tcb->m_segmentSize ) && (tcb->m_cWnd < tcb->m_ssThresh)))
	  {
		  NS_LOG_INFO ("Entering Slow Start");
		   return TcpNewReno::SlowStart (tcb, segmentsAcked);
	  }
	  else if (m_maxSsThresh*tcb->m_segmentSize < tcb->m_cWnd && tcb->m_cWnd < tcb->m_ssThresh)
	  {
		//return LimitedSlowStart(tcb, segmentsAcked);
		if(uint64_t(m_bottleneckBandwidth) > 0.1*m_senderBandwidth.GetBitRate()) //Capacity exapnsion scenario
		{
			return SlowStart(tcb, segmentsAcked);
		}
		
		else //(m_bottleneckBandwidth < 0.1*senderBandwidth) //Capacity reduction scenario
		{
			return LimitedSlowStart(tcb, segmentsAcked);
		}
	  }
    }

  return 0;
}

void
TcpNewReno::PktsAcked (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked,
                     const Time& rtt)
{
	NS_LOG_FUNCTION (this << tcb << segmentsAcked << rtt);
	m_cWndInSegments = tcb->GetCwndInSegments ();

	if(m_cWndInSegments < 90)
	{
		if (m_isStart)
		  {
			  m_maxSent = tcb->m_highTxMark;
			  m_isStart = false;
		  }
		else
		  {
			  if (m_firstPacketInPair)
				{
					if (tcb->m_lastAckedSeq > m_maxSent)
					{
						//This is first packet of packet pair
						
						if(m_firstSample)   //Checking if this is first sample
						{
							m_candidateRtt1 = rtt;
							m_minRtt1 = rtt;
						}
						m_sampleRtt1 = rtt;
						m_firstPacketInPair = false;
						m_maxSent = tcb->m_highTxMark;
					}
				}
			  else //Second packet in the probing packet pair
				{
					//this is second packet of packet pair
					
					if(m_firstSample)   //This is second packet of first ever sample
						{
							m_candidateRtt2 = rtt;
							m_minRtt2 = rtt;
							m_firstSample = false;
						}
						
					m_sampleRtt2 = rtt;
					m_firstPacketInPair = true;
					
					/* Calculation of Candidate Rtt - The best sample seen till now */
					{
						if(m_sampleRtt1 <= m_minRtt1)
						{
							if(m_sampleRtt1 == m_minRtt1)
							{
								if(m_sampleRtt2 < m_minRtt2)
								{
									m_candidateRtt2 = m_sampleRtt2;
									m_minRtt2 = m_sampleRtt2;
								}
							}
							else //(m_sampleRtt1 < m_minRtt1)
							{
								m_candidateRtt1 = m_sampleRtt1;
								m_candidateRtt2 = m_sampleRtt2;
								m_minRtt1 = m_sampleRtt1;

								if(m_sampleRtt2 < m_minRtt2)
								{
									m_minRtt2 = m_sampleRtt2;
								}
							}
						}
						else //(m_sampleRtt1 > m_minRtt1)
						{
							if(m_sampleRtt2 < m_minRtt2)
							{
								m_minRtt2 = m_sampleRtt2;
							}
						}
					}
				}
			}
		}
		else if(m_cWndInSegments == 90)
		{
			m_dispersion = m_candidateRtt2.GetDouble() - m_candidateRtt1.GetDouble();
			m_bottleneckBandwidth = (tcb->m_segmentSize*8)/m_dispersion; //in bits/s
		}
}

/**
 * \brief NewReno congestion avoidance
 *
 * During congestion avoidance, cwnd is incremented by roughly 1 full-sized
 * segment per round-trip time (RTT).
 *
 * \param tcb internal congestion state
 * \param segmentsAcked count of segments acked
 */
void
TcpNewReno::CongestionAvoidance (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);

  if (segmentsAcked > 0)
    {
      double adder = static_cast<double> (tcb->m_segmentSize * tcb->m_segmentSize) / tcb->m_cWnd.Get ();
      adder = std::max (1.0, adder);
      tcb->m_cWnd += static_cast<uint32_t> (adder);
      NS_LOG_INFO ("In CongAvoid, updated to cwnd " << tcb->m_cWnd <<
                   " ssthresh " << tcb->m_ssThresh);
    }
}

/**
 * \brief Try to increase the cWnd following the NewReno specification
 *
 * \see SlowStart
 * \see CongestionAvoidance
 *
 * \param tcb internal congestion state
 * \param segmentsAcked count of segments acked
 */

void
TcpNewReno::IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
 NS_LOG_FUNCTION (this << tcb << segmentsAcked);


  if (tcb->m_cWnd < tcb->m_ssThresh)
    {
      segmentsAcked = CapStart (tcb, segmentsAcked);
    }

  if (tcb->m_cWnd >= tcb->m_ssThresh)
    {
      CongestionAvoidance (tcb, segmentsAcked);
    }
}



std::string
TcpNewReno::GetName () const
{
  return "TcpNewReno";
}

uint32_t
TcpNewReno::GetSsThresh (Ptr<const TcpSocketState> state,
                         uint32_t bytesInFlight)
{
  NS_LOG_FUNCTION (this << state << bytesInFlight);

  return std::max (2 * state->m_segmentSize, bytesInFlight / 2);
}

Ptr<TcpCongestionOps>
TcpNewReno::Fork ()
{
  return CopyObject<TcpNewReno> (this);
}

} // namespace ns3
