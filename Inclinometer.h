
#ifndef PUSHTOGO_INCLINOMETER_H_
#define PUSHTOGO_INCLINOMETER_H_

class Inclinometer {
public:
	Inclinometer(){
	}
	virtual ~Inclinometer(){
	}

	/**
	 * @return Axis tilt on the RA. Positive means it is tilt on the same direction as RA axis positive direction
	 * This function should return immediately!
	 */
	virtual double getTilt() {
		return 0;
	}

	/**
	 * Issue command to refresh data from the sensor
	 */
	virtual void refresh() {
	}

};

#endif /* PUSHTOGO_INCLINOMETER_H_ */
