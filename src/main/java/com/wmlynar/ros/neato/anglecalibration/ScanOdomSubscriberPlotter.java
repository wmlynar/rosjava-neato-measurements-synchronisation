/*
 * Copyright (C) 2014 woj.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.wmlynar.ros.neato.anglecalibration;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

import org.apache.commons.net.nntp.NewGroupsOrNewsQuery;
import org.jfree.ui.RefineryUtilities;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import com.wmlynar.ros.neato.utils.XyTimePlotter;

import nav_msgs.Odometry;
import sensor_msgs.LaserScan;

/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class ScanOdomSubscriberPlotter extends AbstractNodeMain {

	private Subscriber<nav_msgs.Odometry> odomSubscriber;
	private Subscriber<sensor_msgs.LaserScan> scanSubscriber;
	private XyTimePlotter plotter;
	
	private double bias1 = 0;
	private boolean isBias1Set = false;
	private double bias2 = 0;
	private boolean isBias2Set = false;
	
	private double distance = 0;
	private double prevX;
	private double prevY;
	private boolean isPrevOdomSet = false;
	private boolean isPrevScanSet = false;
	
	private boolean paused = false;
	private double prevOdomTimestamp;
	private double prevScanTimestamp;
	private double prevOdom;
	private float prevScan;
	
	private AverageFilter averageOdom = new AverageFilter();
	private AverageFilter averageScan = new AverageFilter();

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("neato/scanOdomPlotter");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		
		plotter = new XyTimePlotter("Scan and Odom");
		RefineryUtilities.centerFrameOnScreen(plotter);
		plotter.setVisible(true);
		
		plotter.setMaximumXRange(5);
		
		plotter.addKeyListener(new KeyListener() {
			
			@Override
			public void keyTyped(KeyEvent e) {
				pauseUnpause();
			}
			
			@Override
			public void keyReleased(KeyEvent e) {
			}
			
			@Override
			public void keyPressed(KeyEvent e) {
			}
		});
		
		odomSubscriber = connectedNode.newSubscriber("odom", nav_msgs.Odometry._TYPE);
		odomSubscriber.addMessageListener(new MessageListener<nav_msgs.Odometry>() {
			@Override
			public void onNewMessage(nav_msgs.Odometry message) {
				try {
					onOdomMessage(message);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}

		});

		scanSubscriber = connectedNode.newSubscriber("base_scan", sensor_msgs.LaserScan._TYPE);
		scanSubscriber.addMessageListener(new MessageListener<sensor_msgs.LaserScan>() {
			@Override
			public void onNewMessage(sensor_msgs.LaserScan message) {
				try {
					onScanMessage(message);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}
	
	public void pauseUnpause() {
		if(!paused) {
			paused = true;
		} else {
			// reset bias
			isBias1Set = false;
			isBias2Set = false;
			paused = false;
		}
	}
	
	private void onOdomMessage(Odometry message) {
		double timestamp = message.getHeader().getStamp().toSeconds();
		
		if(timestamp-prevOdomTimestamp < 0.2) {
			return;
		}
		
		double value = 180 / Math.PI * Utils.fromQuaternionToYaw(message.getPose().getPose().getOrientation());
		
		if(!isPrevOdomSet) {
			prevOdom = value;
			prevOdomTimestamp = timestamp;
			isPrevOdomSet = true;
			return;
		}

		float d = (float) ((value-prevOdom)/(timestamp-prevOdomTimestamp));
		
		prevOdom = value;
		prevOdomTimestamp = timestamp;

		averageOdom.add(d);
		value = (float) averageOdom.get();
//		value = d;
		
		if(value<-80) {
			int i=0;
		}
		
		if(!isBias1Set) {
			bias1 = value;
			distance = 0;
			isBias1Set = true;
			averageOdom.reset();
		}
		//value -= bias1;
		if(!paused) {
			plotter.addValues("odom",timestamp,value);
			Utils.logCsv("target/odom",timestamp,value);
		}
	}
	
	private void onScanMessage(LaserScan message) {
		double timestamp = message.getHeader().getStamp().toSeconds();

		if(timestamp-prevScanTimestamp < 0.2) {
			return;
		}
		
		float[] ranges = message.getRanges();
		float minRange = 0.05f; // message.getRangeMin();
		float maxRange = 5.f; // message.getRangeMax();
		float value = 359 - Utils.getAngleOfNearest(ranges,minRange, maxRange, -1);
		
		if(!isPrevScanSet) {
			prevScan = value;
			prevScanTimestamp = timestamp;
			isPrevScanSet = true;
			return;
		}
		
		float d = (float) ((value-prevScan)/(timestamp-prevScanTimestamp));
		
		prevScan = value;
		prevScanTimestamp = timestamp;

		averageScan.add(d);
		value = (float) averageScan.get();
//		value = d;
		
		if(value > 40 || value < -40) {
			return;
		}
		
		if(!isBias2Set) {
			prevScanTimestamp = timestamp;
			bias2 = value;
			isBias2Set = true;
			averageScan.reset();
		}
		//value-=bias2;
		if(!paused) {
			plotter.addValues("scan",timestamp,value);
			Utils.logCsv("target/scan",timestamp,value);
		}
	}

}

