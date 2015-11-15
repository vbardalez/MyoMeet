// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
// This sample illustrates how to interface with multiple Myo armbands and distinguish between them.
#define _USE_MATH_DEFINES
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include <windows.h>
#include <myo/myo.hpp>
class MyoMeeterDL : public myo::DeviceListener {
public:
	// Every time Myo Connect successfully pairs with a Myo armband, this function will be called.
	//
	// You can rely on the following rules:
	//  - onPair() will only be called once for each Myo device
	//  - no other events will occur involving a given Myo device before onPair() is called with it
	//
	// If you need to do some kind of per-Myo preparation before handling events, you can safely do it in onPair().
	void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
	{
		// Print out the MAC address of the armband we paired with.
		// The pointer address we get for a Myo is unique - in other words, it's safe to compare two Myo pointers to
		// see if they're referring to the same Myo.
		// Add the Myo pointer to our list of known Myo devices. This list is used to implement identifyMyo() below so
		// that we can give each Myo a nice short identifier.
		knownMyos.push_back(myo);
		knownPoses.push_back(new myo::Pose());
		knownPitches.push_back(NULL);
		// Now that we've added it to our list, get our short ID for it and print it out.
		std::cout << "Paired with " << myo << "." << std::endl;
	}

	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
	{
		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;
		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
		// Convert the floating point angles in radians to a scale from 0 to 18

		knownPitches[identifyMyo(myo) - 1] = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 180);

		std::cout << "Myo " << 1 << " switched to orientation " << knownPitches[0] << ". ";
		std::cout << "Myo " << 2 << " switched to orientation " << knownPitches[1] << "." << std::endl;
	}

	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
	{
		myo->unlock(myo::Myo::unlockHold);
		knownPoses[identifyMyo(myo) - 1] = &pose;
		std::cout << "Myo " << identifyMyo(myo) << " switched to pose " << *knownPoses[identifyMyo(myo) - 1] << "." << std::endl;
	}
	void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
	{
		myo->unlock(myo::Myo::unlockHold);
		std::cout << "Myo " << identifyMyo(myo) << " has connected." << std::endl;
	}
	void onDisconnect(myo::Myo* myo, uint64_t timestamp)
	{
		std::cout << "Myo " << identifyMyo(myo) << " has disconnected." << std::endl;
	}
	// This is a utility function implemented for this sample that maps a myo::Myo* to a unique ID starting at 1.
	// It does so by looking for the Myo pointer in knownMyos, which onPair() adds each Myo into as it is paired.
	size_t identifyMyo(myo::Myo* myo) {
		// Walk through the list of Myo devices that we've seen pairing events for.
		for (size_t i = 0; i < knownMyos.size(); ++i) {
			// If two Myo pointers compare equal, they refer to the same Myo device.
			if (knownMyos[i] == myo) {
				return i + 1;
			}
		}
		return 0;
	}
	// We store each Myo pointer that we pair with in this list, so that we can keep track of the order we've seen
	// each Myo and give it a unique short identifier (see onPair() and identifyMyo() above).
	std::vector<myo::Myo*> knownMyos;
	std::vector<myo::Pose*> knownPoses;
	std::vector<int> knownPitches;
};


int main(int argc, char** argv)
{
	try {
		myo::Hub hub("com.example.multiple-myos");
		// Instantiate the PrintMyoEvents class we defined above, and attach it as a listener to our Hub.
		MyoMeeterDL myoMeeter;
		hub.addListener(&myoMeeter);
		while (1) {
			// Process events for 10 milliseconds at a time.
			hub.run(100);
		}
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}
