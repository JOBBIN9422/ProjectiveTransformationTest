using System;
using SixLabors.Primitives;
using SixLabors.ImageSharp;
using System.Numerics;
using SixLabors.ImageSharp.Processing;

namespace TransformTest
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");

            using (var baseImage = Image.Load("baseImage.png"))
            using (var projectImage = Image.Load("projectTest.png"))
            using (var bobRossImage = Image.Load("bobross.jpg"))
            {
                //Point topLeft = new Point(613, 186);
                //Point topRight = new Point(866, 101);
                //Point bottomRight = new Point(850, 538);
                //Point bottomLeft = new Point(612, 432);

                Point topLeft = new Point(297, 22);
                Point topRight = new Point(490, 5);
                Point bottomRight = new Point(493, 213);
                Point bottomLeft = new Point(304, 194);

                //Rectangle sourceRect = new Rectangle(0, 0, projectImage.Width, projectImage.Height);

                Matrix4x4 transformMat = TransformHelper.ComputeTransformMatrix(projectImage.Width, projectImage.Height, topLeft, topRight, bottomLeft, bottomRight);
                Matrix4x4 transformMat2 = TransformHelper.CalculateProjectiveTransformationMatrix(projectImage.Width, projectImage.Height, topLeft, topRight, bottomLeft, bottomRight);

                //projectImage.Mutate(x => x.Transform(sourceRect, transformMat, KnownResamplers.Lanczos3));
                projectImage.Mutate(x => x.Transform(new ProjectiveTransformBuilder().AppendMatrix(transformMat)));
                projectImage.Save("result.png");

                bobRossImage.Mutate(x => x.DrawImage(projectImage, new Point(0, 0), 1.0f));
                bobRossImage.Save("projected.png");
            }

        }

    }
}
