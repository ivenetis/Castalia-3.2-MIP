#include "ILS_MIP_MH.h"

Define_Module ( ILS_MIP_MH );

/******************************************************************************/

void ILS_MIP_MH::startup()
{
	ID			= atoi ( SELF_NETWORK_ADDRESS );
	sampleInterval		= par ( "sampleInterval" );
	bytesCollected		= par ( "bytesCollected" );
	codeSize		= par ( "codeSize" );
	createMAPI		= par ( "createMAPI" );
	DijkstraType		= par ( "DijkstraType" );
	a			= par ( "a" );
	instantiationDelay	= par ( "instantiationDelay" );
	processingDelay		= par ( "processingDelay" );
	aggregationEnergy	= par ( "aggregationEnergy" );
	initialEnergy		= resMgrModule->par ( "initialEnergy" );
	numberOfSensors		= getParentModule()->getParentModule()->par ( "numNodes" );
	THRESHOLD		= 10;
	spentEnergy		= 0.0;
	if ( isSink ) {
		double	PLd0, d0, pathLossExponent, sensitivity;
		int	i = 0;

		prevNodeID		= 0;
		Info[ID][ID].nodeID	= 0;
		Info[ID][ID].xCoord	= mobilityModule->getLocation().x;
		Info[ID][ID].yCoord	= mobilityModule->getLocation().y;
		trace() << fixed << setprecision( 2 ) << " SINK: Position is x= " << mobilityModule->getLocation().x << " and y= " << mobilityModule->getLocation().y;

		PLd0 = getParentModule()->getParentModule()->getSubmodule("wirelessChannel")->par ( "PLd0" );
		d0   = getParentModule()->getParentModule()->getSubmodule("wirelessChannel")->par ( "d0" );
		pathLossExponent = getParentModule()->getParentModule()->getSubmodule("wirelessChannel")->par ( "pathLossExponent" );

		for (list <RXmode_type>::iterator it = radioModule->getRXmodeListBegin(); it != radioModule->getRXmodeListEnd(); it++) {
			if (it->name.compare("IDEAL") == 0) {
				sensitivity = it->sensitivity;
				break;
			}
		}

		listSize = radioModule->getTxLevelListSize();

		TransmissionLevel = (double *)malloc(listSize * sizeof(double));
		if (TransmissionLevel == NULL) {
			trace() << "Could not allocate memory for TransmissionLevel";
			exit(0);
		}

		TransmissionPower = (double *)malloc(listSize * sizeof(double));
		if (TransmissionPower == NULL) {
			trace() << "Could not allocate memory for TransmissionPower";
			exit(0);
		}

		TransmissionRange = (double *)malloc(listSize * sizeof(double));
		if (TransmissionRange == NULL) {
			trace() << "Could not allocate memory for TransmissionRange";
			exit(0);
		}

		for (list <TxLevel_type>::iterator it = radioModule->getTxLevelListBegin(); it != radioModule->getTxLevelListEnd(); it++, i++) {
			TransmissionLevel[i] = it->txOutputPower;
			TransmissionPower[i] = it->txPowerConsumed;
			TransmissionRange[i] = d0 * pow(10.0, (TransmissionLevel[i] - sensitivity - PLd0) / (10.0 * pathLossExponent));
			trace() << "Transmission range is " << setw( 5 ) << TransmissionRange[i] << " for transmission level " << TransmissionLevel[i];
		}

		setTimer ( REQUEST_POSITIONS, 0.5 );
	} else {
		prevNodeID		= NO_ID;
		trace() << fixed << setprecision(2) << "!SINK: Position is x= " << mobilityModule->getLocation().x << " and y= " << mobilityModule->getLocation().y;
		setTimer ( REQUEST_SAMPLE, sampleInterval );
	}
}

/******************************************************************************/

/*
 * Triggered when a packet is received
 */
void ILS_MIP_MH::fromNetworkLayer ( ApplicationPacket * genericPacket, const char *source, double rssi, double lqi )
{
	string packetName ( genericPacket->getName() );

	if ( packetName.compare ( REQUEST_POSITIONS_PACKET ) == 0 ) {
		PosPkt		*rcvPacket = check_and_cast<PosPkt *>(genericPacket);

#if defined(LOG_ENABLED)
		trace() << "Received [" << packetName << "] from " << source;
#endif
		/*
		 * If this node has not already sent it's position.
		 */
		if ( prevNodeID == NO_ID ) {
			prevNodeID = atoi ( source );

			PosPktData	Fill;
			PosPkt		*newPktSend = new PosPkt ( SEND_POSITION_PACKET, APPLICATION_PACKET );
			Fill.nodeID = ID;
			Fill.xCoord = mobilityModule->getLocation().x;
			Fill.yCoord = mobilityModule->getLocation().y;
			newPktSend->setExtraData ( Fill );
			newPktSend->setByteLength ( sizeof ( Fill ) );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
			toNetworkLayer ( newPktSend, source );

			PosPkt	*newPktRequest = new PosPkt ( REQUEST_POSITIONS_PACKET, APPLICATION_PACKET );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
			toNetworkLayer ( newPktRequest, BROADCAST_NETWORK_ADDRESS );
		}
	}
	else if ( packetName.compare ( SEND_POSITION_PACKET ) == 0 ) {
		PosPkt		*rcvPacket = check_and_cast<PosPkt *>(genericPacket);
		PosPktData	Sender = rcvPacket->getExtraData();

#if defined(LOG_ENABLED)
		trace() << "Received [" << packetName << "] from " << source;
#endif
		/*
		 * If the packet is received from the sink, update the information for the node from where the packet was originally sent.
		 */
		if ( isSink ) {
			/*
			 * Information is updated only if it arrives in time.
			 * Otherwise, the information is discarded, since the sink might
			 * already have calculated the distances and sorted the information.
			 */
			if ( reported != REPORT_END_OF_TURN ) {
				Info[Sender.nodeID][Sender.nodeID].nodeID = Sender.nodeID;
				Info[Sender.nodeID][Sender.nodeID].xCoord = Sender.xCoord;
				Info[Sender.nodeID][Sender.nodeID].yCoord = Sender.yCoord;
				sensorsReported++;
				reported = REPORT_RECEIVED;
			}
			/*
			 * prevNodeID could be NO_ID if an END_OF_TURN_PACKET arrived in the meantime.
			 * Normally though (if WAIT_FOR_POSITIONS_INTERVAL is large enough) this should never happen.
			 */
		} else if ( prevNodeID != NO_ID ) {
			stringstream	prevNodeIDss;
			prevNodeIDss << prevNodeID;
			string		prevNodeIDstr = prevNodeIDss.str();

			PosPkt	*newPkt = new PosPkt ( SEND_POSITION_PACKET, APPLICATION_PACKET );
			newPkt->setExtraData ( Sender );
			newPkt->setByteLength ( sizeof ( Sender ) );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
			toNetworkLayer ( newPkt, prevNodeIDstr.c_str() );
		}
	}
	else if ( packetName.compare ( END_OF_TURN_PACKET ) == 0 ) {
		PosPkt		*rcvPacket = check_and_cast<PosPkt *>(genericPacket);

#if defined(LOG_ENABLED)
		trace() << "Received [" << packetName << "] from " << source;
#endif
		if ( ( prevNodeID != NO_ID ) && ( !isSink ) ) {
			startEnergy = resMgrModule->getSpentEnergy();

			prevNodeID = NO_ID;

			PosPkt	*newPkt = new PosPkt ( END_OF_TURN_PACKET, APPLICATION_PACKET );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
			toNetworkLayer ( newPkt, BROADCAST_NETWORK_ADDRESS );
		}
	} else if ( packetName.compare ( MA_PACKET ) == 0 ) {
		MA_Pkt		*rcvPacket = check_and_cast<MA_Pkt *>(genericPacket);
		MA_PktData	Sender = rcvPacket->getExtraData();

#if defined(LOG_ENABLED)
		trace() << "Received [" << packetName << "] from " << source;
#endif
		if ( isSink && ( Sender.nextNodeID == Sender.nodesInPath ) ) {
			endTime = simTime().dbl();
			trace() << "Mobile agent returned to sink from node " << source << " with a packet of size " << Sender.currentSize << "!";
		} else {
			Path_t	*Path = getPathStart(Sender);

			if ( Path[Sender.nextNodeID - 1].collect == 1 ) {
				Sender.currentSize += bytesCollected;

				trace() << "Data collected from node " << Path[Sender.nextNodeID - 1].nodeID;
			} else {
				trace() << "Only wandering through node " << Path[Sender.nextNodeID - 1].nodeID;
			}

			resMgrModule->consumeEnergy( aggregationEnergy );
			spentEnergy = resMgrModule->getSpentEnergy();

			stringstream	nextNodeIDss;
			nextNodeIDss << Path[Sender.nextNodeID].nodeID;
			string		nextNodeIDstr = nextNodeIDss.str();

#if defined(LOG_ENABLED)
			trace() << "Sending to node " << nextNodeIDstr << " with level " << Path[Sender.nextNodeID].level << " a message with size " << Sender.currentSize;
#endif
			MA_Pkt		*MA = new MA_Pkt ( MA_PACKET, APPLICATION_PACKET );
			Sender.nextNodeID++;
			MA->setExtraData ( Sender );
			MA->setByteLength ( Sender.currentSize );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, Path[Sender.nextNodeID - 1].level ) );
			toNetworkLayer ( MA, nextNodeIDstr.c_str() );
		}
	}
}

/******************************************************************************/

void ILS_MIP_MH::timerFiredCallback ( int index )
{
	switch ( index ) {

	case REQUEST_SAMPLE: {
		if ( !isSink ) {
			setTimer ( REQUEST_SAMPLE, sampleInterval );
			requestSensorReading();
		}
		break;
	}

	case REQUEST_POSITIONS: {
		sensorsReported	= 1;
		reported = NO_REPORT_RECEIVED;

		for ( int i = 1; i < numberOfSensors; i++ ) {
			Info[i][i].nodeID = NO_ID;
		}

		setTimer ( WAIT_FOR_POSITIONS, WAIT_FOR_POSITIONS_INTERVAL );

		PosPkt *newPkt = new PosPkt ( REQUEST_POSITIONS_PACKET, APPLICATION_PACKET );
		toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
		toNetworkLayer ( newPkt, BROADCAST_NETWORK_ADDRESS );
		break;
	}

	case SEND_PACKET: {
		/*
		 * Send the mobile agent out.
		 */
		stringstream	nextNodeIDss;
		nextNodeIDss << Path[Fill.nextNodeID].nodeID;
		string		nextNodeIDstr = nextNodeIDss.str();

#if defined(LOG_ENABLED)
		trace() << "Sending to node " << nextNodeIDstr << " with level " << Path[Fill.nextNodeID].level << " a message with size " << Fill.currentSize;
#endif
		Fill.nextNodeID++;
		MA = new MA_Pkt ( MA_PACKET, APPLICATION_PACKET );
		MA->setExtraData ( Fill );
		MA->setByteLength ( Fill.currentSize );
		toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, Path[Fill.nextNodeID - 1].level ) );
		toNetworkLayer ( MA, nextNodeIDstr.c_str() );

		setTimer( FIND_PATH, SEND_PACKET_DELAY );

		break;
	}

	case FIND_PATH: {
		double	cost, minCost, travelDistance;
		int	i, j, k, l, s, v, currentNode, hops, nodesInPath, prevNodesInPath, intermediateNodes, nextNode;
		vector<int>::iterator it_i, it_j, it_k, it_l, min_it_i, min_it_j;
		vector< vector<int> >::iterator	it_vector_i, min_it_vector_i;

		if ( savedS > 0 ) {
			goto OnlySendNextMA;
		}

#if 1
		it_vector_i = I.begin();
		while ( !Vleft.empty() ) {

			minCost = INFINITY;

			for ( it_i = Vleft.begin(); it_i != Vleft.end(); it_i++ ) {
//				trace() << "Calculating cost for node " << *it_i;

				for ( it_j = it_vector_i->begin(); it_j != it_vector_i->end() - 1; it_j++ ) {
					cost = 0.0;
					hops = 0;
					for ( it_k = it_vector_i->begin(); it_k != it_vector_i->end() - 1; it_k++ ) {
						if ( it_j != it_k ) {
							cost += (codeSize + hops * bytesCollected) * Info[*it_k][*(it_k + 1)].cost;
							hops++;
//							trace() << "    1.  From " << *it_k << " to " << *(it_k + 1);
						} else {
							cost += (codeSize + hops * bytesCollected) * Info[*it_k][*it_i].cost;
							hops++;
							cost += (codeSize + hops * bytesCollected) * Info[*it_i][*(it_k + 1)].cost;
							hops++;
//							trace() << "    2.  From " << *it_k << " to " << *it_i;
//							trace() << "    3.  From " << *it_i << " to " << *(it_k + 1);
						}
					}
//					trace() << "  Cost calculated is " << cost << ", minimum cost is " << minCost;
					if ( cost < minCost) {
						minCost  = cost;
						min_it_j = it_j + 1;
						min_it_i = it_i;
					}
				}
			}

//			trace() << "Minimum cost calculated is " << minCost << " for node " << *min_it_i;

			it_vector_i->insert(min_it_j, *min_it_i);
			Vleft.erase( min_it_i );

			it_vector_i++;
			if ( it_vector_i == I.end() ) {
				it_vector_i = I.begin();
			}
		}
#else
//		vector< vector<int> >::iterator	min_it_vector_i;

		while ( !Vleft.empty() ) {
			currentNode = *Vleft.begin();
//			trace() << "currentNode = " << currentNode;

			minCost = INFINITY;
			for ( it_vector_i = I.begin(); it_vector_i != I.end(); it_vector_i++ ) {
//				trace() << "  Looking in itinerary ";
				for ( it_i = it_vector_i->begin(); it_i != it_vector_i->end() - 1; it_i++ ) {
					cost = 0.0;
					hops = 0;
					for ( it_j = it_vector_i->begin(); it_j != it_vector_i->end() - 1; it_j++ ) {
						if ( it_i != it_j ) {
							cost += (codeSize + hops * bytesCollected) * Info[*it_j][*(it_j + 1)].cost;
							hops++;
//							trace() << "    1.  From " << *it_j << " to " << *(it_j + 1);
						} else {
							cost += (codeSize + hops * bytesCollected) * Info[*it_j][currentNode].cost;
							hops++;
							cost += (codeSize + hops * bytesCollected) * Info[currentNode][*(it_j + 1)].cost;
							hops++;
//							trace() << "    2.  From " << *it_j << " to " << currentNode;
//							trace() << "    3.  From " << currentNode << " to " << *(it_j + 1);
						}
					}
//					trace() << "  Cost calculated is " << cost << ", minimum cost is " << minCost;
					if ( cost < minCost) {
						minCost  = cost;
						min_it_i = it_i + 1;
						min_it_vector_i = it_vector_i;
					}
				}
			}
			min_it_vector_i->insert(min_it_i, currentNode);
			Vleft.erase( Vleft.begin() );
		}
#endif

#if defined(LOG_ENABLED)
		trace() << "Itineraries calculated";
		for ( it_vector_i = I.begin(); it_vector_i != I.end(); it_vector_i++ ) {
			for ( it_i = it_vector_i->begin(); it_i != it_vector_i->end() - 1; it_i++ ) {
				trace() << *it_i;
			}
			trace() << "";
		}
#endif

//#define SWAP
//#define OPT_3_OPT
#define OPT_2_OPT

#if defined(OPT_3_OPT)
{
		vector<int>	vc, vopt, vc_temp;
		bool		change;

		for ( it_vector_i = I.begin(); it_vector_i != I.end(); it_vector_i++ ) {
			if (it_vector_i->size() > 3) {
				vc = *it_vector_i;
				minCost = ILSCost( vc );
//				trace() << "Checking next itinerary";
//				trace() << "Initial cost = " << minCost;

				do {
					change = false;
					vopt = vc;
					it_i = vc.begin();
					it_i++;
					for ( ; it_i != vc.end() - 3; it_i++ ) {
//						trace() << "  1st node " << *it_i;
						it_j = it_i;
						it_j++;
						for ( ; it_j != vc.end() - 2; it_j++ ) {
//							trace() << "    2nd node " << *it_j;
							it_k = it_j;
							it_k++;
							for ( ; it_k != vc.end() - 1; it_k++ ) {
//								trace() << "      3rd node " << *it_k;

								if ( it_i == it_j - 1 ) {
									if (it_j == it_k - 1 ) {
//										trace() << "        Case 1. All nodes consecutive.";
										reverse( it_i, it_j + 1 );
//										for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//											trace() << "        " << *it_l;
//										}

										cost = ILSCost( vc );
//										trace() << "        New cost is " << cost;
										if ( cost < minCost ) {
											minCost = cost;
											vopt = vc;
											change = true;
//											trace() << "        Found a case!!!";
										}
										reverse( it_i, it_j + 1 );
//										for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//											trace() << "        " << *it_l;
//										}
//										trace() << "";
									} else {
//										trace() << "        Case 2. 1st and 2nd node consecutive.";
										/*
										 * 1st subcase
										 */
										reverse( it_i, it_k );
//										for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//											trace() << "        " << *it_l;
//										}

										cost = ILSCost( vc );
//										trace() << "        New cost is " << cost;
										if ( cost < minCost ) {
											minCost = cost;
											vopt = vc;
											change = true;
//											trace() << "        Found a case!!!";
										}
										reverse( it_i, it_k );
//										for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//											trace() << "        " << *it_l;
//										}
//										trace() << "";

										/*
										 * 2nd subcase
										 */
										reverse( it_i, it_k );
										reverse( it_i, it_k - 1 );
//										for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//											trace() << "        " << *it_l;
//										}

										cost = ILSCost( vc );
//										trace() << "        New cost is " << cost;
										if ( cost < minCost ) {
											minCost = cost;
											vopt = vc;
											change = true;
//											trace() << "        Found a case!!!";
										}
										reverse( it_i, it_k - 1 );
										reverse( it_i, it_k );
//										for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//											trace() << "        " << *it_l;
//										}
//										trace() << "";

									}
								} else if ( it_j == it_k - 1 ) {
//										trace() << "        Case 3. 2nd and 3rd node consecutive.";
										/*
										 * 1st subcase
										 */
										reverse( it_i, it_k );
//										for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//											trace() << "        " << *it_l;
//										}

										cost = ILSCost( vc );
//										trace() << "        New cost is " << cost;
										if ( cost < minCost ) {
											minCost = cost;
											vopt = vc;
											change = true;
//											trace() << "        Found a case!!!";
										}
										reverse( it_i, it_k );
//										for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//											trace() << "        " << *it_l;
//										}
//										trace() << "";

										/*
										 * 2nd subcase
										 */
										reverse( it_i, it_k );
										reverse( it_i + 1, it_k );
//										for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//											trace() << "        " << *it_l;
//										}

										cost = ILSCost( vc );
//										trace() << "        New cost is " << cost;
										if ( cost < minCost ) {
											minCost = cost;
											vopt = vc;
											change = true;
//											trace() << "        Found a case!!!";
										}
										reverse( it_i + 1, it_k );
										reverse( it_i, it_k );
//										for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//											trace() << "        " << *it_l;
//										}
//										trace() << "";

								} else {
//									trace() << "        Case 4. No consecutive nodes";
									/*
									 * 1st subcase
									 */
									reverse( it_i, it_j );
//									for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//										trace() << "        " << *it_l;
//									}

									cost = ILSCost( vc );
//									trace() << "        New cost is " << cost;
									if ( cost < minCost ) {
										minCost = cost;
										vopt = vc;
										change = true;
//										trace() << "        Found a case!!!";
									}
//									trace() << "";

									/*
									 * 2nd subcase
									 * Swap the already inverted part of the vector with the required part of the vector.
									 */
									vc_temp.clear();
									vc_temp.insert( vc_temp.end(), vc.begin(), it_i );
									vc_temp.insert( vc_temp.end(), it_j, it_k );
									vc_temp.insert( vc_temp.end(), it_i, it_j );
									vc_temp.insert( vc_temp.end(), it_k, vc.end() );
//									for ( it_l = vc_temp.begin(); it_l != vc_temp.end(); it_l++ ) {
//										trace() << "        " << *it_l;
//									}

									cost = ILSCost( vc_temp );
//									trace() << "        New cost is " << cost;
									if ( cost < minCost ) {
										minCost = cost;
										vopt = vc_temp;
										change = true;
//										trace() << "        Found a case!!!";
									}
//									trace() << "";

									/*
									 * Restore original vector.
									 */
									reverse( it_i, it_j );
//									for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//										trace() << "        " << *it_l;
//									}
//									trace() << "";

									/*
									 * 3rd subcase
									 */
									reverse( it_j, it_k );
//									for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//										trace() << "        " << *it_l;
//									}

									cost = ILSCost( vc );
//									trace() << "        New cost is " << cost;
									if ( cost < minCost ) {
										minCost = cost;
										vopt = vc;
										change = true;
//										trace() << "        Found a case!!!";
									}
//									trace() << "";

									/*
									 * 4th subcase
									 * Swap the already inverted part of the vector with the required part of the vector.
									 */
									vc_temp.clear();
									vc_temp.insert( vc_temp.end(), vc.begin(), it_i );
									vc_temp.insert( vc_temp.end(), it_j, it_k );
									vc_temp.insert( vc_temp.end(), it_i, it_j );
									vc_temp.insert( vc_temp.end(), it_k, vc.end() );
//									for ( it_l = vc_temp.begin(); it_l != vc_temp.end(); it_l++ ) {
//										trace() << "        " << *it_l;
//									}

									cost = ILSCost( vc_temp );
//									trace() << "        New cost is " << cost;
									if ( cost < minCost ) {
										minCost = cost;
										vopt = vc_temp;
										change = true;
//										trace() << "        Found a case!!!";
									}
//									trace() << "";

									/*
									 * Restore original vector.
									 */
									reverse( it_j, it_k );
//									for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//										trace() << "        " << *it_l;
//									}
//									trace() << "";

									/*
									 * 5th subcase
									 */
									reverse( it_i, it_j );
									reverse( it_j, it_k );
//									for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//										trace() << "        " << *it_l;
//									}

									cost = ILSCost( vc );
//									trace() << "        New cost is " << cost;
									if ( cost < minCost ) {
										minCost = cost;
										vopt = vc;
										change = true;
//										trace() << "        Found a case!!!";
									}
//									trace() << "";

									/*
									 * 6th subcase
									 * Swap the already inverted parts of the vector.
									 */
									vc_temp.clear();
									vc_temp.insert( vc_temp.end(), vc.begin(), it_i );
									vc_temp.insert( vc_temp.end(), it_j, it_k );
									vc_temp.insert( vc_temp.end(), it_i, it_j );
									vc_temp.insert( vc_temp.end(), it_k, vc.end() );
//									for ( it_l = vc_temp.begin(); it_l != vc_temp.end(); it_l++ ) {
//										trace() << "        " << *it_l;
//									}

									cost = ILSCost( vc_temp );
//									trace() << "        New cost is " << cost;
									if ( cost < minCost ) {
										minCost = cost;
										vopt = vc_temp;
										change = true;
//										trace() << "        Found a case!!!";
									}
//									trace() << "";

									/*
									 * Restore original vector.
									 */
									reverse( it_j, it_k );
									reverse( it_i, it_j );
//									for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//										trace() << "        " << *it_l;
//									}
//									trace() << "";

									/*
									 * 7th subcase
									 */
									vc_temp.clear();
									vc_temp.insert( vc_temp.end(), vc.begin(), it_i );
									vc_temp.insert( vc_temp.end(), it_j, it_k );
									vc_temp.insert( vc_temp.end(), it_i, it_j );
									vc_temp.insert( vc_temp.end(), it_k, vc.end() );
//									for ( it_l = vc_temp.begin(); it_l != vc_temp.end(); it_l++ ) {
//										trace() << "        " << *it_l;
//									}

									cost = ILSCost( vc_temp );
//									trace() << "        New cost is " << cost;
									if ( cost < minCost ) {
										minCost = cost;
										vopt = vc_temp;
										change = true;
//										trace() << "        Found a case!!!";
									}
//									for ( it_l = vc.begin(); it_l != vc.end(); it_l++ ) {
//										trace() << "        " << *it_l;
//									}
//									trace() << "";

								}
							}
						}
					}
					vc = vopt;
				} while (change);
				*it_vector_i = vopt;
			}
		}
}
#endif

#if defined(OPT_2_OPT)
{
		vector<int>	vc, vopt;
		bool		change;

		for ( it_vector_i = I.begin(); it_vector_i != I.end(); it_vector_i++ ) {

			vc = *it_vector_i;
			minCost = ILSCost( vc );
//			trace() << "Checking next itinerary";
//			trace() << "Initial cost = " << minCost;

			do {
				change = false;
				vopt = vc;
				it_i = vc.begin();
				it_i++;
				for ( ; it_i != vc.end() - 2; it_i++ ) {
//					trace() << "  Starting with node " << *it_i;
					it_j = it_i;
					it_j++;
					for ( ; it_j != vc.end() - 1; it_j++ ) {
//						trace() << "    Next node is " << *it_j;
						reverse(it_i, it_j + 1);
//						for ( it_k = vc.begin(); it_k != vc.end(); it_k++ ) {
//							trace() << "      " << *it_k;
//						}

						cost = ILSCost( vc );
//						trace() << "    New cost is " << cost;
						if ( cost < minCost ) {
							minCost = cost;
							vopt = vc;
							change = true;
//							trace() << "    Found a case!!!";
						}
						reverse(it_i, it_j + 1);
//						for ( it_k = vc.begin(); it_k != vc.end(); it_k++ ) {
//							trace() << "      " << *it_k;
//						}
//						trace() << "";
					}
				}
				vc = vopt;
			} while (change);
			*it_vector_i = vopt;
		}
}
#endif

#if defined(SWAP)
{
		int				temp;
		double				cost_i, cost_j, minCost_i, minCost_j;
		bool				change;
		vector<int>			vc_i, vc_j, vopt_i, vopt_j;
		vector< vector<int> >::iterator	it_vector_j;

		do {
			change = false;
			for ( it_vector_i = I.begin(); it_vector_i != I.end() - 1; it_vector_i++ ) {
//				trace() << "Checking next first vector";
//				for ( it_k = it_vector_i->begin(); it_k != it_vector_i->end(); it_k++ ) {
//					trace() << "  " << *it_k;
//				}

				it_vector_j = it_vector_i + 1;
				for ( ; it_vector_j != I.end(); it_vector_j++ ) {
//					trace() << "  Checking next second vector";
//					for ( it_k = it_vector_j->begin(); it_k != it_vector_j->end(); it_k++ ) {
//						trace() << "    " << *it_k;
//					}

					vc_i = *it_vector_i;
					vc_j = *it_vector_j;
					vopt_i = vc_i;
					vopt_j = vc_j;
					minCost_i = ILSCost( vc_i );
					minCost_j = ILSCost( vc_j );
					it_i = vc_i.begin();
					it_i++;
					for ( ; it_i != vc_i.end() - 1; it_i++ ) {
						it_j = vc_j.begin();
						it_j++;
						for ( ; it_j != vc_j.end() - 1; it_j++ ) {
							temp = *it_i;
							*it_i = *it_j;
							*it_j = temp;
							cost_i = ILSCost( vc_i );
							cost_j = ILSCost( vc_j );
//							trace() << "  Exchanging node " << *it_j << " with node " << *it_i << ", minCost = " << minCost_i + minCost_j << ", currentCost = " << cost_i + cost_j;
							if ( cost_i + cost_j < minCost_i + minCost_j ) {
								minCost_i = cost_i;
								minCost_j = cost_j;
								vopt_i = vc_i;
								vopt_j = vc_j;
								change = true;
//								trace() << "    Found a case!";
							}
							temp = *it_i;
							*it_i = *it_j;
							*it_j = temp;
						}
					}
					*it_vector_i = vopt_i;
					*it_vector_j = vopt_j;
				}
			}
		} while (change);
}
#endif

		/*
		 * Now we need to return to the calculation of the cost as requested by the user.
		 * If this is the cost accordying to power consumption then we don't need to do anything further,
		 * as this is already the inormation we have calculated during initializtion.
		 * Otherwise, we need to recalculate.
		 */
		if ( DijkstraType == 1) {
			for ( i = 0; i < numberOfSensors; i++ ) {
				DijkstraDistance ( i, MinVect, PrevVect );
				for ( j = 0; j < numberOfSensors; j++ ) {
					Info[i][j].cost = MinVect[j];
				}
			}
		}

OnlySendNextMA:
		/*
		 * Finally, we iterate over all vectors to create the path that each MA has to follow.
		 */
		for ( s = savedS; s < I.size(); s++ ) {
			trace() << "Traversing nodes in vector " << s;

			Path = getPathStart(Fill);
			nodesInPath = 0;
			currentNode = 0;
			Info[currentNode][currentNode].used = true;

			I[s].erase( I[s].begin() );
			for ( it_i = I[s].begin(); it_i != I[s].end(); it_i++ ) {
				nextNode = *it_i;
				if ( Info[currentNode][nextNode].cost <= MaxTransmissionCost ) {
					Path[nodesInPath].nodeID   = nextNode;
					Path[nodesInPath].collect  = true;
					Path[nodesInPath].level    = Info[currentNode][nextNode].level;
					if ( createMAPI == true ) {
						fprintf ( MAPIfd, "%d\t%d\n", 0, currentNode );
					}
					nodesInPath++;
					trace() << "Next hop is from node " << currentNode << " to node " << nextNode << " with level= " << Info[currentNode][nextNode].level;
				} else {
					trace() << "Next hop is from node " << currentNode << " to node " << nextNode << ". Path to follow:";
					/*
					 * Dijkstra's algorithm returns the path to be followed in the reverse order.
					 * Firstly, we count how many nodes we have to visit to reach the destination node.
					 * Secondly, we reverse again their order when inserting them into the path to be followed by the mobile agent.
					 * Thirdly, we setup correctly the required transmission power level to move from one node to the next one.
					 */
					intermediateNodes = 0;
					for ( v = nextNode; v != currentNode; v = Info[currentNode][v].prev ) {
						intermediateNodes++;
					}

					l = nodesInPath + intermediateNodes - 1;
					for ( v = nextNode; v != currentNode; v = Info[currentNode][v].prev ) {
						Path[l].nodeID   = v;
						Path[l].collect  = false;	// Do not collect information from intermediate nodes
						l--;
						nodesInPath++;
					}
					k = 0;
					for ( l = l + 1; l < nodesInPath; l++) {
						Path[l].level = Info[Path[l - 1].nodeID][Path[l].nodeID].level;
						if ( createMAPI == true ) {
							fprintf ( MAPIfd, "%d\t%d\n", k, Path[l - 1].nodeID );
							k = 1;
						}
						trace() << "  " << Path[l].nodeID << ", " << Path[l].level;
					}
					Path[nodesInPath - 1].collect = true;	// Collect information from destination node
				}
				Info[nextNode][nextNode].used = true;
				currentNode = nextNode;
			}

			/*
			 * We reached the last available node. Now find the shortest path back to the sink.
			 *
			 * If required, the information is written to a MAPI file. The return path is
			 * distinguished by the previous two cases by marking the nodes of the
			 * return path with a "2".
			 */
			prevNodesInPath = nodesInPath;
			intermediateNodes = 0;
			for ( v = 0; v != nextNode; v = Info[currentNode][v].prev ) {
				intermediateNodes++;
			}

			l = nodesInPath + intermediateNodes - 1;
			for ( v = 0; v != nextNode; v = Info[currentNode][v].prev ) {
				Path[l].nodeID  = v;
				Path[l].collect = false;
				/*
				 * Since we are going to visit nodes when sending back to the sink the aqcuired data,
				 * we will collect data from these nodes during the return trip. Therefore, any previous
				 * visit to the nodes of the return path do not require information to be collected.
				 */
				for ( i = 0; i < prevNodesInPath; i++ ) {
					if ( ( Path[i].nodeID == v ) && ( Path[i].collect == true) ) {
						Path[i].collect = false;
						Path[l].collect  = true;
					}
				}
				l--;
				nodesInPath++;
			}

			/*
			 * The previous procedure will also collect information from the sink, which is not required.
			 */
			Path[nodesInPath - 1].collect = false;

			trace() << "Path back to sink:";
			k = 0;
			for ( l = l + 1; l < nodesInPath; l++) {
				Path[l].level = Info[Path[l - 1].nodeID][Path[l].nodeID].level;
				if ( createMAPI == true ) {
					fprintf ( MAPIfd, "%d\t%d\n", k, Path[l - 1].nodeID );
					k = 2;
				}
				trace() << "  " << Path[l].nodeID << ", " << Path[l].level;
			}

			trace() << "Path starting from sink:";
			intermediateNodes = 0;
			travelDistance = Info[0][Path[0].nodeID].distance;
			totalTravelDistance += Info[0][Path[0].nodeID].distance;
			for ( i = 0; i < nodesInPath; i++ ) {
				trace() << "  " << setw( 3 ) << Path[i].nodeID << ", level = " << setw( 6 ) << Path[i].level << ", collect = " << Path[i].collect;
				if ( i < nodesInPath - 1 ) {
					travelDistance += Info[Path[i].nodeID][Path[i + 1].nodeID].distance;
					totalTravelDistance += Info[Path[i].nodeID][Path[i + 1].nodeID].distance;

					if ( Path[i].collect == 0 ) {
						intermediateNodes++;
						totalIntermediateNodes++;
					}
				}
			}

			totalItineraries++;
			totalNodes += ( nodesInPath - 1 );
			if ( intermediateNodes > maxIntermediateNodes ) {
				maxIntermediateNodes = intermediateNodes;
				maxIntermediateNodesItinerary = nodesInPath;
			}
			if ( nodesInPath == maxItineraryWithIntermediateNodes1 ) {
				if ( nodesInPath - intermediateNodes > maxItineraryWithoutIntermediateNodes1 ) {
					maxItineraryWithoutIntermediateNodes1 = nodesInPath - intermediateNodes;
				}
			} else if ( nodesInPath > maxItineraryWithIntermediateNodes1 ) {
				maxItineraryWithIntermediateNodes1    = nodesInPath;
				maxItineraryWithoutIntermediateNodes1 = nodesInPath - intermediateNodes;
			}
			if ( nodesInPath - intermediateNodes == maxItineraryWithoutIntermediateNodes2 ) {
				if ( nodesInPath > maxItineraryWithIntermediateNodes2 ) {
					maxItineraryWithIntermediateNodes2    = nodesInPath;
				}
			} else if ( nodesInPath - intermediateNodes > maxItineraryWithoutIntermediateNodes2 ) {
				maxItineraryWithIntermediateNodes2    = nodesInPath;
				maxItineraryWithoutIntermediateNodes2 = nodesInPath - intermediateNodes;
			}
			if ( travelDistance > maxItineraryTravelDistance ) {
				maxItineraryTravelDistance = travelDistance;
			}

			Fill.currentSize = getInitSize(nodesInPath);
			Fill.nextNodeID  = 0;
			Fill.nodesInPath = nodesInPath;

			time.push_back( ( savedS + 1 ) * instantiationDelay + nodesInPath * processingDelay );

			savedS = s + 1;		// Restart from next node.

			setTimer( SEND_PACKET, SEND_PACKET_DELAY + instantiationDelay );
			return;
		}

		if ( createMAPI == true ) {
			fprintf ( MAPIfd, "%d\t%d\n", 0, 0 );

			fprintf ( MAPIfd, "\n" );

			fprintf ( MAPIfd, "[Coordinates]\n" );
			for ( i = 0; i < numberOfSensors; i++ ) {
				VirtualMobilityManager *nodeMobilityModule = check_and_cast<VirtualMobilityManager*>(getParentModule()->getParentModule()->getSubmodule("node",i)->getSubmodule("MobilityManager"));
				double x = nodeMobilityModule->getLocation().x;
				double y = nodeMobilityModule->getLocation().y;
				fprintf ( MAPIfd, "%f\t%f\t%d\t\"%d\"\n", x, y, ( Info[i][i].used == true ? 1 : 0 ), i );
			}

			fclose ( MAPIfd );
		}

		break;
	}

	case WAIT_FOR_POSITIONS: {
		if ( ( sensorsReported == numberOfSensors ) || ( reported == NO_REPORT_RECEIVED ) ) {
			int		i, j, k;
			double		field_x = getParentModule()->getParentModule()->par ( "field_x" );
			double		field_y = getParentModule()->getParentModule()->par ( "field_y" );
			stringstream	out;

//			if ( sensorsReported == 1 ) {
//				trace() << "Fatal error: No node reported its position.";
//				exit(0);
//			}

			reported = REPORT_END_OF_TURN;

			PosPkt	*newPkt = new PosPkt ( END_OF_TURN_PACKET, APPLICATION_PACKET );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
			toNetworkLayer ( newPkt, BROADCAST_NETWORK_ADDRESS );

			if ( createMAPI == true ) {
				MAPIfd = fopen ( "ILS_MIP_MH.mapi", "w" );

				fprintf ( MAPIfd, "[Field]\n" );
				fprintf ( MAPIfd, "%f\t%f\n\n", field_x, field_y );

				fprintf ( MAPIfd, "[Nodes]\n" );
				fprintf ( MAPIfd, "%d\n\n", numberOfSensors );
			}

			/*
			 * Setup information according to the data received from every node.
			 * The structure Info[i][j] describes the information between nodes i and j.
			 * More specifically, the information kept is as follows:
			 *   - If both nodes have reported their position, their distance can be calculated. Otherwise, their distance is set to INFINITY.
			 *   - If the distance previously calculated is less or equal to the maximum transmission range of a node,
			 *     the two nodes can exchange information directly. In this case, the smallest transmission level that can cover the distance is selected.
			 *     This is to preserve as much as possible the energy of the nodes.
			 *   - If the distance previously calculated is larger than the maximum transmission range of a node,
			 *     the two nodes cannot directly exchange information and the corresponding transmission power and level are set to INFINITY.
			 */
#if 1
			sensorsReported = numberOfSensors;
			for ( i = 0; i < numberOfSensors; i++ ) {
				VirtualMobilityManager *nodeMobilityModule = check_and_cast<VirtualMobilityManager*>(getParentModule()->getParentModule()->getSubmodule("node",i)->getSubmodule("MobilityManager"));
				Info[i][i].xCoord = nodeMobilityModule->getLocation().x;
				Info[i][i].yCoord = nodeMobilityModule->getLocation().y;
				Info[i][i].distance = 0.0;	// Distance from myself
				Info[i][i].level    = 0.0;
				Info[i][i].power    = 0.0;
				Info[i][i].cost     = 0.0;
				Info[i][i].prev     = -1;
				Info[i][i].used     = false;

				for ( j = 0; j < i; j++ ) {
					Info[i][j].distance = sqrt ( pow ( Info[i][i].xCoord - Info[j][j].xCoord, 2) + pow ( Info[i][i].yCoord - Info[j][j].yCoord, 2) );
					Info[j][i].distance = Info[i][j].distance;
				}
			}
#else
			for ( i = 0; i < numberOfSensors; i++ ) {
				Info[i][i].distance = 0.0;	// Distance from myself
				Info[i][i].cost     = 0.0;
				Info[i][i].prev     = -1;
				Info[i][i].used     = false;
				for ( j = 0; j < i; j++ ) {
					if ( ( Info[i][i].nodeID != NO_ID ) && ( Info[j][j].nodeID != NO_ID ) ) {
						Info[i][j].distance = sqrt ( pow ( Info[i][i].xCoord - Info[j][j].xCoord, 2) + pow ( Info[i][i].yCoord - Info[j][j].yCoord, 2) );
						Info[j][i].distance = Info[i][j].distance;
					} else {
						Info[i][j].distance = INFINITY;
						Info[j][i].distance = INFINITY;
					}

					if (Info[i][j].distance <= MAX_TRANSMISSION_RANGE ) {
						k = listSize - 1;
						while ( Info[i][j].distance > TransmissionRange[k] ) {
							k--;
						}
						Info[i][j].level = TransmissionLevel[k];
						Info[j][i].level = Info[i][j].level;
						Info[i][j].power = TransmissionPower[k];
						Info[j][i].power = Info[i][j].power;
					} else {
						Info[i][j].level = INFINITY;
						Info[j][i].level = INFINITY;
						Info[i][j].power = INFINITY;
						Info[j][i].power = INFINITY;
					}
				}
			}
#endif

			/*
			 * Now that the distances between nodes have been correctly set up, we can use Diskstra's algorithm
			 * to calculate the minimum cost to send information from one node to another. If the communication
			 * cannot be performed directly, we save the path that has to be followed.
			 */
			for ( i = 0; i < numberOfSensors; i++ ) {
				if ( DijkstraType == 1) {
					DijkstraDistance ( i, MinVect, PrevVect );
					MaxTransmissionCost = MAX_TRANSMISSION_RANGE;
				} else if ( DijkstraType == 2 ) {
					DijkstraPower ( i, MinVect, PrevVect );
					MaxTransmissionCost = MAX_TRANSMISSION_POWER;
				} else {
					trace() << "Unknown type (=" << DijkstraType << ") of Dijkstra algorithm.";
				}
				for ( j = 0; j < numberOfSensors; j++ ) {
					Info[i][j].cost = MinVect[j];
					Info[i][j].prev = PrevVect[j];
				}
			}

#if 1
			for ( i = 1; i < numberOfSensors; i++ ) {
				if ( Info[i][0].prev == -1) {
					sensorsReported--;
					Info[i][i].nodeID = NO_ID;
					for ( j = 0; j < numberOfSensors; j++ ) {
						if ( i != j ) {
							Info[i][j].distance = INFINITY;
							Info[j][i].distance = INFINITY;
							Info[i][j].level    = INFINITY;
							Info[j][i].level    = INFINITY;
							Info[i][j].power    = INFINITY;
							Info[j][i].power    = INFINITY;
							Info[i][j].cost     = INFINITY;
							Info[j][i].cost     = INFINITY;
							Info[i][j].prev     = -1;
							Info[j][i].prev     = -1;
						}
					}
				} else {
					Info[i][i].nodeID = i;
					for ( j = 0; j < i; j++ ) {
						if (Info[i][j].distance <= MAX_TRANSMISSION_RANGE ) {
							k = listSize - 1;
							while ( Info[i][j].distance > TransmissionRange[k] ) {
								k--;
							}
							Info[i][j].level = TransmissionLevel[k];
							Info[j][i].level = Info[i][j].level;
							Info[i][j].power = TransmissionPower[k];
							Info[j][i].power = Info[i][j].power;
						} else {
							Info[i][j].level = INFINITY;
							Info[j][i].level = INFINITY;
							Info[i][j].power = INFINITY;
							Info[j][i].power = INFINITY;
						}
					}
				}
			}
#endif

			/*
			 * Especially for ILS we need the cost according to the transmission power required.
			 */
			for ( i = 0; i < numberOfSensors; i++ ) {
				DijkstraPower ( i, MinVect, PrevVect );
				for ( j = 0; j < numberOfSensors; j++ ) {
					Info[i][j].cost = MinVect[j];
				}
			}

			if ( createMAPI == true ) {
				fprintf ( MAPIfd , "[CommunicationCost]\n" );
				for ( i = 0; i < numberOfSensors; i++ ) {
					for ( j = 0; j < numberOfSensors - 1; j++ ) {
						if ( Info[i][j].cost != INFINITY ) {
							fprintf ( MAPIfd, "%f\t", Info[i][j].cost );
						} else {
							fprintf ( MAPIfd, "Infinity\t" );	// The tool that reads this information is written in Java, which uses "Infinity" as the literal to describe infinity.
						}
					}
					if ( Info[i][numberOfSensors - 1].cost != INFINITY ) {
						fprintf ( MAPIfd, "%f\n", Info[i][numberOfSensors - 1].cost );
					} else {
						fprintf ( MAPIfd, "Infinity\n" );
					}
				}
				fprintf ( MAPIfd, "\n" );
			}

#if defined(LOG_ENABLED)
			for ( i = 0; i < numberOfSensors; i++ ) {
				out.str("");
				for ( j = 0; j < numberOfSensors; j++ ) {
					out << fixed << setprecision( 2 ) << setw ( 6 ) << Info[i][j].distance << " ";
				}
				trace() << out.str();
			}

			trace() << "";

			for ( i = 0; i < numberOfSensors; i++ ) {
				out.str("");
				for ( j = 0; j < numberOfSensors; j++ ) {
					out << fixed << setprecision( 2 ) << setw ( 6 ) << Info[i][j].level << " ";
				}
				trace() << out.str();
			}

			trace() << "";

			for ( i = 0; i < numberOfSensors; i++ ) {
				out.str("");
				for ( j = 0; j < numberOfSensors; j++ ) {
					out << fixed << setprecision( 2 ) << setw ( 6 ) << Info[i][j].power << " ";
				}
				trace() << out.str();
			}

			trace() << "";

			for ( i = 0; i < numberOfSensors; i++ ) {
				out.str("");
				for ( j = 0; j < numberOfSensors; j++ ) {
					out << fixed << setprecision( 2 ) << setw ( 6 ) << Info[i][j].cost << " ";
				}
				trace() << out.str();
			}

			trace() << "";

			for ( i = 0; i < numberOfSensors; i++ ) {
				out.str("");
				for ( j = 0; j < numberOfSensors; j++ ) {
					out << fixed << setprecision( 2 ) << setw ( 6 ) << Info[i][j].prev << " ";
				}
				trace() << out.str();
			}
#endif

			/*
			 * Clear the vector that holds the nodes that don't belong to an itinerary yet.
			 */
			Vleft.clear();

			/*
			 * Clear the vector that holds the vectors of nodes that belong to each itinerary.
			 */
			for ( i = 0; i < I.size(); i++ ) {
				I[i].clear();
			}
			I.clear();

			/*
			 * Find the number of nodes that are within transmission range of the sink.
			 */
			j = 0;
			for ( i = 1; i < numberOfSensors; i++ ) {
				if ( Info[0][i].distance < a * MAX_TRANSMISSION_RANGE ) {
					j++;
				}
			}

			/*
			 * Resize the vector of itineraries to hold as many itineraries as there are sensors within transmission range of the sink.
			 */
			I.resize( j );

			/*
			 * Find the nodes that are within transmission range of the sink and insert them into the corresponding vector.
			 */
			for ( i = 1, j = 0; i < numberOfSensors; i++ ) {
				if ( Info[0][i].distance < a * MAX_TRANSMISSION_RANGE ) {
					I[j].push_back( 0 );
					I[j].push_back( i );
					I[j].push_back( 0 );
					j++;
				} else if ( Info[i][i].nodeID != NO_ID ) {
					Vleft.push_back( i );
				}
			}

#if defined(LOG_ENABLED)
			for ( i = 0; i < I.size(); i++ ) {
				trace() << "Nodes in itinerary " << i;
				for ( vector<int>::iterator it = I[i].begin(); it != I[i].end(); it++ ) {
					trace() << "  " << *it;
				}
			}

			trace() << "Nodes not yet in an itinerary";
			for ( vector<int>::iterator it = Vleft.begin(); it != Vleft.end(); it++ ) {
					trace() << "  " << *it;
			}
#endif

			savedS = 0;
			totalNodes = 0;
			totalItineraries = 0;
			totalIntermediateNodes = 0;
			maxIntermediateNodes = 0;
			maxIntermediateNodesItinerary = 0;
			maxItineraryWithIntermediateNodes1 = 0;
			maxItineraryWithIntermediateNodes2 = 0;
			maxItineraryWithoutIntermediateNodes1 = 0;
			maxItineraryWithoutIntermediateNodes2 = 0;
			maxItineraryTravelDistance = 0.0;
			totalTravelDistance = 0.0;

			/*
			 * Now calculate the path that the mobile agent has to follow.
			 * First, the current node finds the node with the smallest cost to send the data.
			 * If the next node is within the transmission range of the current node, the information is sent directly.
			 * Otherwise, the information from running Dijkstra's algorithm is used to calculate an alternative path
			 * to get to the next node.
			 *
			 * If required, the information is written to a MAPI file. The two cases described above
			 * (directly sending data or through an alternative path) are marked with a "0" or a "1"
			 * respectively in the MAPI file.
			 */

			if ( createMAPI == true ) {
				fprintf ( MAPIfd, "[Path]\n" );
			}

			time.clear();

			startTime = simTime().dbl() + 0.1;

			setTimer( FIND_PATH, 0.1 );

//			setTimer ( REQUEST_POSITIONS, REQUEST_POSITIONS_INTERVAL );
		} else {
			reported = NO_REPORT_RECEIVED;
			setTimer ( WAIT_FOR_POSITIONS, WAIT_FOR_POSITIONS_INTERVAL );
		}
		break;
	}
	}
}

/******************************************************************************/

void ILS_MIP_MH::handleSensorReading ( SensorReadingMessage * sensorMsg )
{
	double	sensValue = sensorMsg->getSensedValue();

	if ( !isSink ) {
		if ( sensValue > THRESHOLD ) {
#if defined(LOG_ENABLED)
			trace() << "Sensed value is " << sensValue << " from sensor at (" << mobilityModule->getLocation().x << ", " << mobilityModule->getLocation().y << ")";
#endif
		}
	}
}

/******************************************************************************/

void ILS_MIP_MH::finishSpecific()
{
	list<double>::iterator it, max_it, secondMax_it;
	double max, secondMax;

	if ( isSink ) {
//		if ( time.size() == 0 ) {
//			trace() << setprecision ( 8 ) << "Total time = " << 0.0;
//		} else
		if ( time.size() == 1 ) {
			it = time.begin();
			trace() << setprecision ( 8 ) << "Total time = " << ( endTime - startTime - 2 * SEND_PACKET_DELAY + *it );
		} else {
			max_it = time.begin();
			for ( it = time.begin(); it != time.end(); it++ ) {
				if ( *it > *max_it ) {
					max_it = it;
				}
			}
			max = *max_it;

			time.erase(max_it);

			secondMax_it = time.begin();
			for ( it = time.begin(); it != time.end(); it++ ) {
				if ( *it > *secondMax_it ) {
					secondMax_it = it;
				}
			}
			secondMax = *secondMax_it;

			trace() << setprecision ( 8 ) << "Total time                       = " << ( endTime - startTime - 2 * SEND_PACKET_DELAY + ( max - secondMax ) );
		}
		trace() << setprecision ( 8 ) << "Total itineraries                = " << totalItineraries;
		trace() << setprecision ( 8 ) << "Total travel distance            = " << totalTravelDistance;
		trace() << setprecision ( 8 ) << "Maximum travel distance          = " << maxItineraryTravelDistance;
		trace() << setprecision ( 8 ) << "Total nodes reported             = " << sensorsReported - 1;
		trace() << setprecision ( 8 ) << "Total nodes visited              = " << totalNodes;
		trace() << setprecision ( 8 ) << "Total intermediate nodes         = " << totalIntermediateNodes;
		trace() << setprecision ( 8 ) << "Maximum intermediate nodes added = " << maxIntermediateNodes;
		trace() << setprecision ( 8 ) << "  Total nodes in itinerary       = " << maxIntermediateNodesItinerary;
		trace() << setprecision ( 8 ) << "Maximum itinerary with intermediate nodes:";
		trace() << setprecision ( 8 ) << "  With intermediate nodes        = " << maxItineraryWithIntermediateNodes1;
		trace() << setprecision ( 8 ) << "  Without intermediate nodes     = " << maxItineraryWithoutIntermediateNodes1;
		trace() << setprecision ( 8 ) << "Maximum itinerary without intermediate nodes:";
		trace() << setprecision ( 8 ) << "  With intermediate nodes        = " << maxItineraryWithIntermediateNodes2;
		trace() << setprecision ( 8 ) << "  Without intermediate nodes     = " << maxItineraryWithoutIntermediateNodes2;
	} else {
		if ( Info[ID][ID].nodeID != NO_ID ) {
			if (spentEnergy == 0.0 ) {
				spentEnergy = startEnergy;
			}
			trace() << "Energy spent = " << spentEnergy - startEnergy;
		} else {
			trace() << "Energy spent = " << 0.0;
		}
	}
}

/******************************************************************************/

void ILS_MIP_MH::DijkstraDistance ( int source, vector<double> &MinVect, vector<int> &PrevVect )
{
	set<pair<double, int> > vertex_queue;

	double distance_through_u;

	MinVect.clear();
	MinVect.resize ( numberOfSensors, INFINITY );
	MinVect[source] = 0.0;
	PrevVect.clear();
	PrevVect.resize ( numberOfSensors, -1 );

	vertex_queue.insert ( make_pair ( MinVect[source], source ) );

	while ( !vertex_queue.empty() ) {
		double	dist = vertex_queue.begin()->first;
		int	u    = vertex_queue.begin()->second;
		vertex_queue.erase ( vertex_queue.begin() );

		// Visit each edge exiting u
		for ( int v = 0; v < numberOfSensors; v++ ) {
			if ( ( Info[u][v].distance <= MAX_TRANSMISSION_RANGE ) && ( u != v ) ) {
				distance_through_u = dist + Info[u][v].distance;
				if ( distance_through_u < MinVect[v] ) {
					vertex_queue.erase ( make_pair ( MinVect[v], v ) );

					MinVect[v] = distance_through_u;
					PrevVect[v] = u;
					vertex_queue.insert ( make_pair ( MinVect[v], v ) );

				}
			}
		}
	}
}

/******************************************************************************/

void ILS_MIP_MH::DijkstraPower ( int source, vector<double> &MinVect, vector<int> &PrevVect )
{
	set<pair<double, int> > vertex_queue;

	double power_through_u;

	MinVect.clear();
	MinVect.resize ( numberOfSensors, INFINITY );
	MinVect[source] = 0.0;
	PrevVect.clear();
	PrevVect.resize ( numberOfSensors, -1 );

	vertex_queue.insert ( make_pair ( MinVect[source], source ) );

	while ( !vertex_queue.empty() ) {
		double	dist = vertex_queue.begin()->first;
		int	u    = vertex_queue.begin()->second;
		vertex_queue.erase ( vertex_queue.begin() );

		// Visit each edge exiting u
		for ( int v = 0; v < numberOfSensors; v++ ) {
			if ( ( Info[u][v].power <= MAX_TRANSMISSION_POWER ) && ( u != v ) ) {
				power_through_u = dist + Info[u][v].power;
				if ( power_through_u < MinVect[v] ) {
					vertex_queue.erase ( make_pair ( MinVect[v], v ) );

					MinVect[v] = power_through_u;
					PrevVect[v] = u;
					vertex_queue.insert ( make_pair ( MinVect[v], v ) );

				}
			}
		}
	}
}

/******************************************************************************/

double ILS_MIP_MH::ILSCost( vector<int> vec )
{
	double	cost = 0.0;
	int	hops = 0;

	for ( vector<int>::iterator it = vec.begin(); it != vec.end() - 1; it++ ) {
		cost += (codeSize + hops * bytesCollected) * Info[*it][*(it + 1)].cost;
		hops++;
	}

	return( cost );
}

/******************************************************************************/
