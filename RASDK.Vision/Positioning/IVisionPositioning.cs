using System.Drawing;

namespace RASDK.Vision.Positioning
{
    /// <summary>
    /// Vision positioning interface.
    /// </summary>
    public interface IVisionPositioning
    {
        /// <summary>
        /// 世界座標偏移。
        /// </summary>
        PointF WorldOffset { get; set; }

        /// <summary>
        /// Get point of world by pixel of image.
        /// </summary>
        /// <param name="pixelX"></param>
        /// <param name="pixelY"></param>
        /// <param name="worldX"></param>
        /// <param name="worldY"></param>
        void ImageToWorld(double pixelX, double pixelY, out double worldX, out double worldY);

        /// <summary>
        /// Get point of world by pixel of image.
        /// </summary>
        /// <param name="pixel"></param>
        /// <returns>The point of world.</returns>
        PointF ImageToWorld(PointF pixel);
    }
}