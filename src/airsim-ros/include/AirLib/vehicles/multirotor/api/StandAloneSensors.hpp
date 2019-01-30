#ifndef air_StandAloneSensors_hpp
#define air_StandAloneSensors_hpp


#include "sensors/imu/ImuSimple.hpp"
#include "sensors/Barometer/BarometerSimple.hpp"
#include "sensors/Magnetometer/MagnetometerSimple.hpp"
#include "common/Common.hpp"
#include <thread>
#include <ostream>

#include "common/CommonStructs.hpp"

namespace msr {
    namespace airlib {
        class StandALoneSensors {
        public:
            ImuData generateImuStaticData(float period)
            {
				ImuData result;
                auto kinematics = Kinematics::State::zero();
                msr::airlib::Environment::State initial_environment(kinematics.pose.position, GeoPoint());
                msr::airlib::Environment environment(initial_environment);
                environment.reset();

                ImuSimple imu;
                imu.initialize(&kinematics, &environment);
                imu.reset();

                TTimeDelta last = Utils::getTimeSinceEpochSecs();
               
                const auto& output = imu.getOutput();
				result.linear_acceleration = output.linear_acceleration;
				result.angular_velocity = output.angular_velocity;
                std::this_thread::sleep_for(std::chrono::duration<double>(period - (Utils::getTimeSinceEpochSecs() - last)));
                last = Utils::getTimeSinceEpochSecs();
                environment.update();
                imu.update();
				return result;
            }

            BarometerData generateBarometerStaticData(float period)
            {
				 
				BarometerData result;
				GeoPoint loc;
                auto kinematics = Kinematics::State::zero();
                msr::airlib::Environment::State initial_environment(kinematics.pose.position, loc);
                msr::airlib::Environment environment(initial_environment);
                environment.reset();

                BarometerSimple baro;
                baro.initialize(&kinematics, &environment);
                baro.reset();

                TTimeDelta last = Utils::getTimeSinceEpochSecs();
                const auto& output = baro.getOutput();
				result.altitude = output.altitude;
				result.pressure = output.pressure;
                std::this_thread::sleep_for(std::chrono::duration<double>(period - (Utils::getTimeSinceEpochSecs() - last)));
                last = Utils::getTimeSinceEpochSecs();
                environment.update();
                baro.update();

				return result;
				
            }

            static MagnetometerData generateMagnetometer2D(float period)
            {
				MagnetometerData result;
				GeoPoint loc;
                TTimeDelta last = Utils::getTimeSinceEpochSecs();
				float yawStart = 0;
				bool ccw = false;
                for (float direction = 0; direction < 5; direction++) 
				{

					float yaw = yawStart;
					float yawDelta = (direction * M_PIf / 2.0f);
					if (ccw) 
					{
						yaw -= yawDelta;
					}
				    else
				    {
						yaw += yawDelta;
				    }

					  auto kinematics = Kinematics::State::zero();
					  kinematics.pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
					  msr::airlib::Environment::State initial_environment(kinematics.pose.position, loc);
					  msr::airlib::Environment environment(initial_environment);
					  environment.reset();

					  MagnetometerSimple mag;
					  mag.initialize(&kinematics, &environment);
					  mag.reset();

					 const auto& output = mag.getOutput();
					 result.magnetic_field_body = output.magnetic_field_body;       
					 std::this_thread::sleep_for(std::chrono::duration<double>(period - (Utils::getTimeSinceEpochSecs() - last)));
					 last = Utils::getTimeSinceEpochSecs();
					 environment.update();
					 mag.update();
                    
                }
				return result;
            }
        
		};

    }
}
#endif