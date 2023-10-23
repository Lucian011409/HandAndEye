#include "EndoscopeAndNDI.h"
#include "HandEyeCalibration.h"


int main()
{
	
	//EndoscopeAndNDI a(imageSize,12,squareSize);
	//a.CaptureData();
	//a.runAndSave();
	//_CrtSetBreakAlloc(7135);
	

	HandEye h(Size(640, 480));
	h.en.setSquareSize(6);
	if (h.runEndoscopeAndNDI())
	{
		if (h.runHandEyeCalibration())
		{
			//h.backProjectionError();
		}
		//if (h.runHandEyeCalibrationHtoE())
		//{
		//	//h.backProjectionError();
		//}
	}
	//EndoscopeAndNDI en;

	//NDI i;

	

	system("pause");
	return 0;
}