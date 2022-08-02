using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Emgu.CV;
using Emgu.CV.Util;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Aruco;

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
    }
}