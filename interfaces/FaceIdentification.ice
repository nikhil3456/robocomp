
#ifndef FACEIDENTIFICATION_ICE
#define FACEIDENTIFICATION_ICE

module RoboCompFaceIdentification
{
	sequence<byte> ImgType;

	struct TImage
	{
		int width;
		int height;
		int depth;
		ImgType image;
	};

	interface FaceIdentification
	{
		idempotent void addNewFace(TImage faceImg, string faceLabel);
		idempotent void getFaceLabels(TImage faceImg, out string faceLabel);  
		idempotent void deleteLabel(string faceLabel);
	};
};

#endif
