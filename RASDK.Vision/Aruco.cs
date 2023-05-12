using Emgu.CV;
using Emgu.CV.Aruco;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System.Drawing;

namespace RASDK.Vision
{
    public class Aruco
    {
        public Dictionary Dictionary = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_50);

        public int Detect(Image<Bgr, byte> image,
                          out VectorOfVectorOfPointF corners,
                          out VectorOfInt ids)
        {
            corners = new VectorOfVectorOfPointF();
            ids = new VectorOfInt();
            ArucoInvoke.DetectMarkers(image, Dictionary, corners, ids, DetectorParameters.GetDefault());
            return (int)ids.Length;
        }

        // bits x bits (per marker) _ number of markers in dict
        public static Emgu.CV.Mat PrintArucoBoard(GridBoard ArucoBoard,
                                                  int markersNumOnXaxis,
                                                  int markersNumOnYaxis,
                                                  int markersLength,
                                                  int markersSeparation)
        {
            // Size of the border of a marker in bits
            int borderBits = 1;

            // Draw the board on a cv::Mat
            Size backGroundImage = new Size();
            Emgu.CV.Mat boardImage = new Emgu.CV.Mat();
            backGroundImage.Width = markersNumOnXaxis * (markersLength + markersSeparation);
            backGroundImage.Height = markersNumOnYaxis * (markersLength + markersSeparation);

            ArucoBoard.Draw(backGroundImage, boardImage, markersSeparation, borderBits);

            // Save the image
            //boardImage.Save("C:\\Download\\arucoboard.png");
            return boardImage;
        }
    }
}