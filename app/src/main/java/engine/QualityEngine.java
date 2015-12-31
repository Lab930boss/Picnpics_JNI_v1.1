package engine;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.PointF;
import android.media.ExifInterface;
import android.media.FaceDetector;
import android.util.Log;

import java.io.IOException;
import java.util.ArrayList;

import model.PhotosTableModel;

public class QualityEngine {

    public static void runQualityEngine() {

        // Return file list that needs to be clustered.
        EngineDBInterface engineDBInterface = new EngineDBInterface();

        ArrayList ClusteredArrayList = engineDBInterface.getClusterIdWithoutQualityScore();
        int size_ClusteredArrayList = ClusteredArrayList.size();
        int MAX_FACES = 5;

        for (int i = 0; i < size_ClusteredArrayList; i++) {

            /////////////////////////////////////////////////////////
            /*                Find unevaluated clusteres            */
            /////////////////////////////////////////////////////////
            int ClusterList = (int) ClusteredArrayList.get(i);
            //int ClusterList= (int)ClusteredArrayList.get(0);

            /*                Find pictures in each cluster            */
            ArrayList QualityFileArray = engineDBInterface.getPhotoListForClusterId(ClusterList);

            double[] score_quality = new double[QualityFileArray.size()];
            for (int k = 0; k < QualityFileArray.size(); k++) {

                PhotosTableModel QualityFileClass = (PhotosTableModel) QualityFileArray.get(k);
                String fname = (String) QualityFileClass.getFile_location();
                String ftime = (String) QualityFileClass.getUpdate_date();
                FaceDetector.Face[] faces;
                int face_count;

                /////////////////////////////////////////////////////////
            /*          Get image orientation and rerotation       */
                /////////////////////////////////////////////////////////
                BitmapFactory.Options bitmap_options = new BitmapFactory.Options();
                bitmap_options.inPreferredConfig = Bitmap.Config.RGB_565;
                bitmap_options.inSampleSize = 4;
                Bitmap bMap = BitmapFactory.decodeFile(fname, bitmap_options);

                /////////////////////////////////////////////////////////
            /*          Get image orientation and rerotation       */
                /////////////////////////////////////////////////////////
                ExifInterface exif = null;
                try {
                    exif = new ExifInterface(fname);
                } catch (IOException e) {
                    e.printStackTrace();
                }
                int orientation = exif.getAttributeInt(ExifInterface.TAG_ORIENTATION,
                        ExifInterface.ORIENTATION_UNDEFINED);

                Bitmap bmRotated = rotateBitmap(bMap, orientation);

                ///////////////////////////////////////////////////////////////
             /*                     Edge detection                        */
                ///////////////////////////////////////////////////////////////

                int imgW3 = bmRotated.getWidth();  // Height
                int imgH3 = bmRotated.getHeight();  // Width
                double[][] sobel_out = new double[imgH3][imgW3];
                sobel_out = edgeDetection_sobel(bmRotated);

                double sum_sobel_out=0;
                for (int x = 1; x < imgH3-1; x++)  // Width
                    for (int y = 1; y < imgW3-1; y++)  //Height
                        sum_sobel_out=sum_sobel_out+sobel_out[x][y];

                sum_sobel_out=100 * sum_sobel_out/(imgH3*imgW3);  // 10 is multiplied to make balance between this score and face detection score

                Log.d("sum_sobel_out=" , String.valueOf(sum_sobel_out));
                /////////////////////////////////////////////////////////
            /*          FAce detection                             */
                /////////////////////////////////////////////////////////
                FaceDetector face_detector = new FaceDetector(
                        bmRotated.getWidth(), bmRotated.getHeight(), MAX_FACES);

                faces = new FaceDetector.Face[MAX_FACES]; // The bitmap must be in 565 format (for now).
                face_count = face_detector.findFaces(bmRotated, faces);
                Log.d("Face_Detection", "Face Count: " + String.valueOf(face_count));


                double avg_dist_from_center = 0;
                double imgW = bmRotated.getWidth();  // Height
                double imgH = bmRotated.getHeight();  // Width

                for (int j = 0; j < face_count; j++) {

                    FaceDetector.Face face = faces[j];
                    PointF myMidPoint = new PointF();
                    face.getMidPoint(myMidPoint);

                    // Top left corner is 0,0 Bottom right corner is Max x Max y
                    float x = myMidPoint.x;
                    float y = myMidPoint.y;


                    double dist_from_center = Math.sqrt((x - imgW / 2.0) * (x - imgW / 2.0) + (y - imgH / 2.0) * (y - imgH / 2.0));

                    float myEyesDistance = face.eyesDistance();
                    if(myEyesDistance>0)   // It means eye is detected. Then we give best score which is minimym distance from the center of the picture.
                        dist_from_center = 0;

                    avg_dist_from_center = avg_dist_from_center + dist_from_center;


                    Log.d("myEyesDistance=", String.valueOf(myEyesDistance));
                }

                if (face_count == 0) {
                    avg_dist_from_center = 0.01 ;

                } else {
                    avg_dist_from_center = 1 - (avg_dist_from_center / (double) face_count)/(Math.sqrt((imgW - imgW / 2.0) * (imgW - imgW / 2.0) + (imgH - imgH / 2.0) * (imgH - imgH / 2.0)));  //Normalized and averaged face loaction with respec to the image center
                }

                Log.d("avg_dist_from_center=", String.valueOf(avg_dist_from_center));
                Log.d("face area=", String.valueOf(face_count * (30*30) / (Math.sqrt((imgW - imgW / 2.0) * (imgW - imgW / 2.0) + (imgH - imgH / 2.0) * (imgH - imgH / 2.0)))));

                score_quality[k] = avg_dist_from_center + sum_sobel_out + face_count * (30*30) / (Math.sqrt((imgW - imgW / 2.0) * (imgW - imgW / 2.0) + (imgH - imgH / 2.0) * (imgH - imgH / 2.0))) ;
                Log.d("score_quality=", String.valueOf(score_quality[k]));
                /////////////////////////////////////////////////////////////////
                /*                          DB update for score                 */
                /////////////////////////////////////////////////////////////////

                engineDBInterface.updateQualityScore(fname, score_quality[k]);

            }

                /*  Sort Score and define rank */

            /////////////////////////////////////////////////////////////////
                /*                          DB update for rank                 */
            /////////////////////////////////////////////////////////////////

            //simEngineDBHelper.updateClusterID(fname,(int)Cluster_cnt);

        }


    }


    /**
     * @param path
     * @param sampleSize 1 = 100%, 2 = 50%(1/2), 4 = 25%(1/4), ...
     * @return
     */
    public static Bitmap getBitmapFromLocalPath(String path, int sampleSize) {
        try {
            BitmapFactory.Options options = new BitmapFactory.Options();
            options.inSampleSize = sampleSize;
            return BitmapFactory.decodeFile(path, options);
        } catch (Exception e) {
            //  Logger.e(e.toString());
        }

        return null;
    }

    public static Bitmap getResizedBitmap(Bitmap srcBitmap, int newWidth, int newHeight) {
        try {
            return Bitmap.createScaledBitmap(srcBitmap, newWidth, newHeight, true);
        } catch (Exception e) {
            //  Logger.e(e.toString());
        }

        return null;
    }

    /**
     * @param bytes
     * @return
     */
    public static Bitmap getBitmapFromBytes(byte[] bytes) {
        try {
            return BitmapFactory.decodeByteArray(bytes, 0, bytes.length);
        } catch (Exception e) {
            //  Logger.e(e.toString());
        }

        return null;
    }

    public static Bitmap rotateBitmap(Bitmap bitmap, int orientation) {

        Matrix matrix = new Matrix();
        switch (orientation) {
            case ExifInterface.ORIENTATION_NORMAL:
                return bitmap;
            case ExifInterface.ORIENTATION_FLIP_HORIZONTAL:
                matrix.setScale(-1, 1);
                break;
            case ExifInterface.ORIENTATION_ROTATE_180:
                matrix.setRotate(180);
                break;
            case ExifInterface.ORIENTATION_FLIP_VERTICAL:
                matrix.setRotate(180);
                matrix.postScale(-1, 1);
                break;
            case ExifInterface.ORIENTATION_TRANSPOSE:
                matrix.setRotate(90);
                matrix.postScale(-1, 1);
                break;
            case ExifInterface.ORIENTATION_ROTATE_90:
                matrix.setRotate(90);
                break;
            case ExifInterface.ORIENTATION_TRANSVERSE:
                matrix.setRotate(-90);
                matrix.postScale(-1, 1);
                break;
            case ExifInterface.ORIENTATION_ROTATE_270:
                matrix.setRotate(-90);
                break;
            default:
                return bitmap;
        }
        try {
            Bitmap bmRotated = Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(), bitmap.getHeight(), matrix, true);
            bitmap.recycle();
            return bmRotated;
        } catch (OutOfMemoryError e) {
            e.printStackTrace();
            return null;
        }
    }


    public static Bitmap bitmapFromArray(int[][] pixels2d){
        int width = pixels2d.length;
        int height = pixels2d[0].length;
        int[] pixels = new int[width * height];
        int pixelsIndex = 0;
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                pixels[pixelsIndex] = pixels2d[i][j];
                pixelsIndex ++;
            }
        }
        return Bitmap.createBitmap(pixels, width, height, Bitmap.Config.ARGB_8888);
    }

    public static double[][] edgeDetection_sobel(Bitmap bmap)
    {

        // double [][]Gx=new double[3][3];
        // double [][]Gy=new double[3][3];
        double [][]Gx={{-1,0,1},{-2,0,2},{-1,0,1}};
        double [][]Gy={{-1,-2,-1},{0,0,0},{1,2,1}};


        /////////////////////////////////////////////////////////
        /*               Gray scale image acquisition          */
        /////////////////////////////////////////////////////////
        double[][] garray, Gx_out, Gy_out, G_out;

        int imgW3 = bmap.getWidth();  // Height
        int imgH3 = bmap.getHeight();  // Width
        garray = new double[imgH3][imgW3];
        Gx_out = new double[imgH3][imgW3];
        Gy_out = new double[imgH3][imgW3];
        G_out = new double[imgH3][imgW3];


        for (int x = 0; x < imgH3; x++)  // Width
        {
            for (int y = 0; y < imgW3; y++)  //Height
            {
                int colour = bmap.getPixel(y, x);
                garray[x][y] =  (double)(0.2989 * (float) Color.red(colour)) +  (0.5870 * (float) Color.green(colour)) +  (0.114 * (float) Color.blue(colour));

            }
        }

        for (int x = 1; x < imgH3-1; x++)  // Width
        {
            for (int y = 1; y < imgW3-1; y++)  //Height
            {

                Gx_out[x][y] = garray[x-1][y-1] * Gx[0][0] + garray[x][y-1] * Gx[1][0] + garray[x+1][y-1] * Gx[2][0]
                        + garray[x-1][y] * Gx[0][1] + garray[x][y] * Gx[1][1] + garray[x+1][y] * Gx[2][1]
                        + garray[x-1][y+1] * Gx[0][2] + garray[x][y+1] * Gx[1][2] + garray[x+1][y+1] * Gx[2][2];

                Gy_out[x][y] = garray[x-1][y-1] * Gy[0][0] + garray[x][y-1] * Gy[1][0] + garray[x+1][y-1] * Gy[2][0]
                        + garray[x-1][y] * Gy[0][1] + garray[x][y] * Gy[1][1] + garray[x+1][y] * Gy[2][1]
                        + garray[x-1][y+1] * Gy[0][2] + garray[x][y+1] * Gy[1][2] + garray[x+1][y+1] * Gy[2][2];

                G_out[x][y] =  Math.sqrt( Math.pow(Gx_out[x][y],2) +  Math.pow(Gy_out[x][y],2) )/255;

            }
        }


        for (int x = 1; x < imgH3-1; x++)  // Width
        {
            for (int y = 1; y < imgW3-1; y++)  //Height
            {
                if (G_out[x][y]>=0.8)
                    G_out[x][y] = 1;
                else
                    G_out[x][y] = 0;
            }
        }


        return G_out;
    }

}
