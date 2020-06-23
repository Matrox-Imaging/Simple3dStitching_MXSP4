//***************************************************************************************/
// 
// File name: Simple3dStiching.cpp  
//
// Synopsis:  This program contains an example of 3D surface registration.
//            It registers two partial point clouds of a 3D object and
//            stitches them into a single complete point cloud.
//            See the PrintHeader() function below for detailed description.
//
// Copyright © Matrox Electronic Systems Ltd., 1992-YYYY.
// All Rights Reserved
//***************************************************************************************/             
#include <mil.h>
#include <math.h>

//-------------------------------------------------------------------------------
// Example description.
//-------------------------------------------------------------------------------
void PrintHeader()
   {
   MosPrintf(MIL_TEXT("[EXAMPLE NAME]\n"));
   MosPrintf(MIL_TEXT("Simple3dStiching\n\n"));

   MosPrintf(MIL_TEXT("[SYNOPSIS]\n"));
   MosPrintf(MIL_TEXT("This example demonstrates how to use the 3D surface registration \n"));
   MosPrintf(MIL_TEXT("to register two partial point clouds of a 3D object and \n"));
   MosPrintf(MIL_TEXT("stitch them into a single complete point cloud.\n"));
   MosPrintf(MIL_TEXT("\n"));

   MosPrintf(MIL_TEXT("[MODULES USED]\n"));
   MosPrintf(MIL_TEXT("Modules used: 3D Registration, Buffer, 3D Image Processing,\n")
             MIL_TEXT("3D Display, 3D Graphics, 3D Geometry and 3D Metrology.\n\n"));
   }

// Utility functions.
bool   CheckForRequiredMILFile      (MIL_CONST_TEXT_PTR FileName);
MIL_ID Alloc3dDisplayId             (MIL_ID MilSystem);

// Enumerators definitions.
enum { eSource = 0, eTarget, eStitched};

// The number of point clouds.
static const MIL_INT NB_POINT_CLOUD = 2;

// Extraction box definitions.
static const MIL_DOUBLE EXTRACTION_BOX_SIZE_X = 170.0;
static const MIL_DOUBLE EXTRACTION_BOX_SIZE_Y = 200.0;
static const MIL_DOUBLE EXTRACTION_BOX_SIZE_Z = -66;

// Expected target location.
static const MIL_DOUBLE BOX_OVERLAP = 0.20;
static const MIL_DOUBLE BOX_USED_OVERLAP = 0.9 * BOX_OVERLAP;

// Registration context controls definitions.
static const MIL_DOUBLE GRID_SIZE = 1.0;
static const MIL_INT    DECIMATION_STEP = 8;
static const MIL_DOUBLE OVERLAP = 95; // %
static const MIL_INT    MAX_ITERATIONS = 100;
static const MIL_DOUBLE RMS_ERROR_RELATIVE_THRESHOLD = 0.5;  // %
static const MIL_INT    ERROR_MINIMIZATION_METRIC = M_POINT_TO_POINT;

// Visualization variables definitions.
static const MIL_INT    NUM_BOX_POINTS = 24; // A 3d cube box has 24 points.
static const MIL_DOUBLE DRAW_BOX_MIN_X = -EXTRACTION_BOX_SIZE_X / 2;
static const MIL_DOUBLE DRAW_BOX_MIN_Y = -EXTRACTION_BOX_SIZE_Y / 2 * BOX_USED_OVERLAP;
static const MIL_DOUBLE DRAW_BOX_MIN_Z =  EXTRACTION_BOX_SIZE_Z / 2;
static const MIL_DOUBLE DRAW_BOX_MAX_X =  EXTRACTION_BOX_SIZE_X / 2;
static const MIL_DOUBLE DRAW_BOX_MAX_Y =  EXTRACTION_BOX_SIZE_Y / 2 * BOX_USED_OVERLAP;
static const MIL_DOUBLE DRAW_BOX_MAX_Z = -EXTRACTION_BOX_SIZE_Z / 2;

// Displays constants.
static const MIL_INT WINDOWS_OFFSET_X = 15;
static const MIL_INT WINDOWS_OFFSET_Y = 40;
static const MIL_INT NB_DISPLAY = 3;
static MIL_CONST_TEXT_PTR DISPLAY_NAMES[NB_DISPLAY] =
   {
   MIL_TEXT("Reference partial point"),
   MIL_TEXT("Target partial point"),
   MIL_TEXT("Stitched point cloud")
   };

// Point clouds information.
// Input data files.
static const MIL_TEXT_CHAR* const FILE_SOURCE_POINT_CLOUD[2] =
   {
      M_IMAGE_PATH MIL_TEXT("/Simple3dStitching/StitchReference.ply"),
      M_IMAGE_PATH MIL_TEXT("/Simple3dStitching/StitchTarget.ply")
   };

static const MIL_INT DISP_3D_SIZE_X = 384;
static const MIL_INT DISP_3D_SIZE_Y = 384;

//-------------------------------------------------------------------------------
// Main.
//-------------------------------------------------------------------------------
int MosMain()
   {
   // Print example information in console.
   PrintHeader();

   auto MilApplication = MappAlloc(M_NULL, M_DEFAULT, M_UNIQUE_ID);

   //Check for required example files.
   if(!CheckForRequiredMILFile(FILE_SOURCE_POINT_CLOUD[0]))
      {
      return -1;
      }
   MIL_ID MilSystem = M_DEFAULT_HOST;

   //-------------------------------------------------------------------------------------------
   // Create the point cloud containers.

   // Allocate 3D point cloud containers.
   MIL_UNIQUE_BUF_ID MilPointCloud[NB_POINT_CLOUD+1];
   MIL_UNIQUE_BUF_ID MilCroppedPointCloud[NB_POINT_CLOUD];

   MilPointCloud[eStitched] = MbufAllocContainer(MilSystem, M_PROC+M_DISP, M_DEFAULT, M_UNIQUE_ID);

   for(MIL_INT i = 0; i < NB_POINT_CLOUD ; ++i)
      MilCroppedPointCloud[i] = MbufAllocContainer(MilSystem, M_PROC + M_DISP, M_DEFAULT, M_UNIQUE_ID);

   // Restore the unorganized point clouds.
   MosPrintf(MIL_TEXT("The reference and target point clouds are being restored..."));
   MilPointCloud[eSource] = MbufRestore(FILE_SOURCE_POINT_CLOUD[0], MilSystem, M_UNIQUE_ID );
   MilPointCloud[eTarget] = MbufRestore(FILE_SOURCE_POINT_CLOUD[1], MilSystem, M_UNIQUE_ID);

   MosPrintf(MIL_TEXT("done.\n\n"));

   //-------------------------------------------------------------------------------
   // Initialize 3D displays that will show the two partial point clouds and the stitched cloud.

   // Initialize displays.
   MIL_ID MilDisplay[NB_DISPLAY];

   for(MIL_INT d = 0; d < NB_DISPLAY; d++)
      {
      // Allocate the display.
      MilDisplay[d] = Alloc3dDisplayId(MilSystem);

      // Some display controls.
      M3ddispControl(MilDisplay[d], M_WINDOW_INITIAL_POSITION_X, (MIL_INT)(d*(WINDOWS_OFFSET_X + DISP_3D_SIZE_X)));
      M3ddispControl(MilDisplay[d], M_SIZE_X, DISP_3D_SIZE_X);
      M3ddispControl(MilDisplay[d], M_SIZE_Y, DISP_3D_SIZE_Y);
 
      // Add titles to displays.
      M3ddispControl(MilDisplay[d], M_TITLE, DISPLAY_NAMES[d]);
      // Adjust view point
      M3ddispSetView(MilDisplay[d], M_AUTO, M_BOTTOM_VIEW, M_DEFAULT, M_DEFAULT,  M_DEFAULT);
      }

   // Re-positionned the stitched cloud's display window.
   M3ddispControl(MilDisplay[eStitched], M_WINDOW_INITIAL_POSITION_X, (MIL_INT)((WINDOWS_OFFSET_X / 2 + DISP_3D_SIZE_X / 2)));
   M3ddispControl(MilDisplay[eStitched], M_WINDOW_INITIAL_POSITION_Y, (MIL_INT)((WINDOWS_OFFSET_Y + DISP_3D_SIZE_Y)));

   MIL_ID MilGraphicList = M_NULL;
   for(MIL_INT p = 0; p < NB_POINT_CLOUD; p++)
      {
      // Display the container.
      MIL_INT64 CloudLabel = M3ddispSelect(MilDisplay[p], MilPointCloud[p],M_SELECT,M_DEFAULT);
      M3ddispInquire(MilDisplay[p], M_3D_GRAPHIC_LIST_ID, &MilGraphicList);
      M3dgraControl(MilGraphicList, CloudLabel, M_COLOR_USE_LUT, M_TRUE);
      M3dgraControl(MilGraphicList, CloudLabel, M_COLOR_COMPONENT, M_COMPONENT_RANGE);
      M3dgraControl(MilGraphicList, CloudLabel, M_COLOR_COMPONENT_BAND, 2);
      }

   // Get the total number of points of the reference point cloud.
   MIL_UNIQUE_3DIM_ID ResultId = M3dimAllocResult(MilSystem, M_STATISTICS_RESULT, M_DEFAULT, M_UNIQUE_ID);
   MIL_INT SourceTotalNbPoints;
   M3dimStat(M_STAT_CONTEXT_NUMBER_OF_POINTS, MilPointCloud[eSource], ResultId, M_DEFAULT );
   M3dimGetResult(ResultId, M_NUMBER_OF_POINTS_VALID, &SourceTotalNbPoints);

   // Define the overlap box.
   MIL_UNIQUE_3DGEO_ID MilBox = M3dgeoAlloc(MilSystem, M_GEOMETRY, M_DEFAULT, M_UNIQUE_ID);
   M3dgeoBox(MilBox, M_CENTER_AND_DIMENSION,
             0.0, 0.0, 0.0,
             EXTRACTION_BOX_SIZE_X, EXTRACTION_BOX_SIZE_Y * BOX_USED_OVERLAP, EXTRACTION_BOX_SIZE_Z,
             M_DEFAULT);

   // Draw the overlap boxes.
   for(MIL_INT i = 0; i < NB_POINT_CLOUD; ++i)
      {
      M3ddispInquire(MilDisplay[i], M_3D_GRAPHIC_LIST_ID, &MilGraphicList);
      MIL_INT64 MilBoxGraphics = M3dgeoDraw3d(M_DEFAULT, MilBox, MilGraphicList, M_DEFAULT, M_DEFAULT);
      M3dgraControl(MilGraphicList, MilBoxGraphics, M_COLOR, M_COLOR_WHITE);
      M3dgraControl(MilGraphicList, MilBoxGraphics, M_APPEARANCE, M_WIREFRAME);
      }

   MosPrintf(MIL_TEXT("The object's reference and target, are displayed using pseudo colors.\n")
             MIL_TEXT("A white box is displayed to show the expected common overlap region\n")
             MIL_TEXT("for both partial point clouds.\n\n"));
   MosPrintf(MIL_TEXT("Press <Enter> to perform the registration.\n"));
   MosGetch();

   //--------------------------------------------------------------------------
   // 3D registration.

   MosPrintf(MIL_TEXT("\tProcessing."));

   // 3D pairwise registration context and result.
   MIL_ID MilRegistrationContext = M3dregAlloc(MilSystem, M_PAIRWISE_REGISTRATION_CONTEXT, M_DEFAULT, M_NULL);
   MIL_ID MilRegistrationResult  = M3dregAllocResult(MilSystem, M_PAIRWISE_REGISTRATION_RESULT, M_DEFAULT, M_NULL);

   // Pairwise registration context controls.
   MIL_ID MilSubsampleContext = M_NULL;
   M3dregInquire(MilRegistrationContext, M_DEFAULT, M_SUBSAMPLE_CONTEXT_ID, &MilSubsampleContext);

   // Subsampling is used to reduce the number of points used during the registration and yield faster results.
   M3dimControl(MilSubsampleContext, M_STEP_SIZE_X, DECIMATION_STEP);
   M3dimControl(MilSubsampleContext, M_STEP_SIZE_Y, DECIMATION_STEP);

   M3dregControl(MilRegistrationContext, M_DEFAULT, M_SUBSAMPLE, M_ENABLE);
   M3dregControl(MilRegistrationContext, M_DEFAULT, M_MAX_ITERATIONS, MAX_ITERATIONS);
   M3dregControl(MilRegistrationContext, M_DEFAULT, M_RMS_ERROR_RELATIVE_THRESHOLD, RMS_ERROR_RELATIVE_THRESHOLD);
   M3dregControl(MilRegistrationContext, M_DEFAULT, M_ERROR_MINIMIZATION_METRIC, ERROR_MINIMIZATION_METRIC);

   // Registration.
   MIL_INT  RegistrationStatus = M_NULL;
   MIL_DOUBLE ComputationTime = 0.0;

   // Set the box to the expected overlap region.
   for(MIL_INT p = 0; p < NB_POINT_CLOUD; p++)
      {
      M3dimCrop(MilPointCloud[p], MilCroppedPointCloud[p], MilBox, M_NULL, M_DEFAULT, M_DEFAULT);
      MosPrintf(MIL_TEXT("."));
      }

   MappTimer(M_TIMER_RESET, M_NULL);

   // Pre-registration with a given overlap.
   M3dregControl(MilRegistrationContext, M_ALL, M_OVERLAP, OVERLAP);
   M3dregCalculate(MilRegistrationContext, MilCroppedPointCloud, NB_POINT_CLOUD,
                   MilRegistrationResult, M_DEFAULT);
   MosPrintf(MIL_TEXT("."));
  
   MIL_ID MilPreregistration = MilRegistrationResult;

   M3dgeoBox(MilBox,
             M_CENTER_AND_DIMENSION,
             0.0, 0.0, 0.0,
             EXTRACTION_BOX_SIZE_X, EXTRACTION_BOX_SIZE_Y * BOX_OVERLAP, EXTRACTION_BOX_SIZE_Z,
             M_DEFAULT);

   // Get the number of points of the source point cloud in the expected overlap region.
   MIL_INT SourceOverlapNbOfPoints;
   MIL_UNIQUE_3DMET_ID MilStatResult = M3dmetAllocResult(MilSystem, M_STATISTICS_RESULT, M_DEFAULT, M_UNIQUE_ID);
   M3dmetStat(M_STAT_CONTEXT_NUMBER, MilPointCloud[eSource], MilBox, MilStatResult, M_SIGNED_DISTANCE_TO_SURFACE, M_LESS_OR_EQUAL, 0, M_NULL, M_DEFAULT);
   M3dmetGetResult(MilStatResult, M_STAT_NUMBER, &SourceOverlapNbOfPoints);

   // Set the box to the expected overlap region.
   for(MIL_INT p = 0; p < NB_POINT_CLOUD; p++)
      {
      M3dimCrop(MilPointCloud[p], MilCroppedPointCloud[p],
                MilBox, M_NULL, M_DEFAULT, M_DEFAULT);
      MosPrintf(MIL_TEXT("."));
      }

   // Set the full model overlap based on the expected overlap between the two point clouds.
   MIL_DOUBLE FullModelOverlap = ((MIL_DOUBLE)SourceOverlapNbOfPoints / SourceTotalNbPoints) * OVERLAP;
   M3dregControl(MilRegistrationContext, M_ALL, M_OVERLAP, FullModelOverlap);

   // Set the pre-registration matrix.
   M3dregSetLocation(MilRegistrationContext, eTarget, eSource, MilPreregistration, M_DEFAULT, M_DEFAULT, M_DEFAULT);

   // Use the full point clouds.
   MosPrintf(MIL_TEXT("."));
   M3dregCalculate(MilRegistrationContext, MilCroppedPointCloud, NB_POINT_CLOUD,
                   MilRegistrationResult, M_DEFAULT);
   MosPrintf(MIL_TEXT("."));

   MappTimer(M_TIMER_READ, &ComputationTime);
   MosPrintf(MIL_TEXT("done\n\n"));

   MosPrintf(MIL_TEXT("The 3D stitching between the two partial point clouds has been performed with \n")
             MIL_TEXT("the help of the points within the expected common overlap regions.\n\n"));

   M3dregGetResult(MilRegistrationResult, eTarget, M_STATUS_REGISTRATION_ELEMENT, &RegistrationStatus);

   // Interpret the result status.
   switch(RegistrationStatus)
      {
      case M_NOT_INITIALIZED:
         MosPrintf(MIL_TEXT("Registration failed: the registration result is not initialized.\n\n"));
         break;

      case M_NOT_ENOUGH_POINT_PAIRS:
         MosPrintf(MIL_TEXT("Registration failed: point clouds are not overlapping.\n\n"));
         break;

      case M_MAX_ITERATIONS_REACHED:
         MosPrintf(MIL_TEXT("Registration reached the maximum number of iterations allowed (%d)\n")
                   MIL_TEXT("in %.2f ms. Resulting fixture may or may not be valid.\n\n"),
                   MAX_ITERATIONS, ComputationTime * 1000);
         break;

      case M_RMS_ERROR_THRESHOLD_REACHED:
      case M_RMS_ERROR_RELATIVE_THRESHOLD_REACHED:
         MIL_DOUBLE RegistrationRmsError;
         M3dregGetResult(MilRegistrationResult, eTarget, M_RMS_ERROR , &RegistrationRmsError);
         MosPrintf(MIL_TEXT("The registration of the two partial point clouds\n")
                   MIL_TEXT("succeeded in %.2f ms with a final RMS error of %f mm.\n\n"),
                   ComputationTime * 1000.0, RegistrationRmsError);
         break;

      default:
         MosPrintf(MIL_TEXT("Unknown registration status.\n\n"));
      }

   //--------------------------------------------------------------------------
   // Stitching

   // Add color to the two clouds.
   for(MIL_INT i = 0; i < NB_POINT_CLOUD; ++i)
      {
      M3ddispControl(MilDisplay[i], M_UPDATE, M_DISABLE);
      MbufFreeComponent(MilPointCloud[i], M_COMPONENT_REFLECTANCE, M_DEFAULT);
      MIL_INT SizeX = MbufInquireContainer(MilPointCloud[i], M_COMPONENT_RANGE, M_SIZE_X, M_NULL);
      MIL_INT SizeY = MbufInquireContainer(MilPointCloud[i], M_COMPONENT_RANGE, M_SIZE_Y, M_NULL);
      MIL_ID MilReflectance = MbufAllocComponent(MilPointCloud[i], 3, SizeX, SizeY, M_UNSIGNED + 8, M_IMAGE + M_PROC + M_DISP, M_COMPONENT_REFLECTANCE, M_NULL);//Colored reflectance

      MbufClear(MilReflectance, (i == 0) ? M_RGB888(135, 165, 235) : M_RGB888(75, 125, 215));
      M3ddispControl(MilDisplay[i], M_UPDATE, M_ENABLE);
      }

   M3dregMerge(MilRegistrationResult, MilPointCloud, 2, MilPointCloud[eStitched], M_NULL, M_DEFAULT);

   // Display stitched point cloud.
   M3ddispSelect(MilDisplay[eStitched], MilPointCloud[eStitched], M_SELECT, M_DEFAULT);

   // Draw a 3D box in the stitched point cloud to show the original overlap regions.
   M3ddispInquire(MilDisplay[eStitched], M_3D_GRAPHIC_LIST_ID, &MilGraphicList);
   MIL_INT64 MilBoxGraphics =
      M3dgraBox(MilGraphicList,
                M_ROOT_NODE, M_BOTH_CORNERS,
                -0.5 * EXTRACTION_BOX_SIZE_X, -0.5 * EXTRACTION_BOX_SIZE_Y * BOX_USED_OVERLAP,  0.5 * EXTRACTION_BOX_SIZE_Z,
                 0.5* EXTRACTION_BOX_SIZE_X ,  0.5 * EXTRACTION_BOX_SIZE_Y * BOX_USED_OVERLAP, -0.5 * EXTRACTION_BOX_SIZE_Z,
               M_DEFAULT, M_DEFAULT);
   M3dgraControl(MilGraphicList, MilBoxGraphics, M_COLOR, M_COLOR_WHITE);
   M3dgraControl(MilGraphicList, MilBoxGraphics, M_APPEARANCE, M_WIREFRAME);

   MosPrintf(MIL_TEXT("The two point clouds have been stitched into a single point cloud.\n")
             MIL_TEXT("The resulting stitched point cloud is displayed.\n")
             MIL_TEXT("A white rectangular box show the transformed overlap region.\n\n"));
   MosPrintf(MIL_TEXT("Press <Enter> to end.\n"));
   MosGetch();

   //--------------------------------------------------------------------------
   // Free MIL objects.
   M3dregFree(MilRegistrationContext);
   M3dregFree(MilRegistrationResult);

   for(MIL_INT d = 0; d < NB_DISPLAY; d++)
      {
      if(MilDisplay[d])
         { M3ddispFree(MilDisplay[d]); }
      }
   return 0;
   }

//--------------------------------------------------------------------------
bool CheckForRequiredMILFile(MIL_CONST_TEXT_PTR FileName)
   {
   MIL_INT FilePresent = M_NO;

   MappFileOperation(M_DEFAULT, FileName, M_NULL, M_NULL, M_FILE_EXISTS, M_DEFAULT, &FilePresent);
   if(FilePresent == M_NO)
      {
      MosPrintf(MIL_TEXT("\n")
                MIL_TEXT("The footage needed to run this example is missing. You need \n")
                MIL_TEXT("to obtain and apply a separate specific update to have it.\n\n"));

      MosPrintf(MIL_TEXT("Press <Enter> to end.\n\n"));
      MosGetch();
      }

   return (FilePresent == M_YES);
   }

//--------------------------------------------------------------------------
// Allocates a 3D display and returns its MIL identifier.  
//--------------------------------------------------------------------------
MIL_ID Alloc3dDisplayId(MIL_ID MilSystem)
   {
   MappControl(M_DEFAULT, M_ERROR, M_PRINT_DISABLE);
   MIL_ID MilDisplay3D = M3ddispAlloc(MilSystem, M_DEFAULT, MIL_TEXT("M_DEFAULT"), M_DEFAULT, M_NULL);
   MappControl(M_DEFAULT, M_ERROR, M_PRINT_ENABLE);

   if(!MilDisplay3D)
      {
      MosPrintf(MIL_TEXT("\n")
                MIL_TEXT("The current system does not support the 3D display.\n")
                MIL_TEXT("Press any key to exit.\n"));
      MosGetch();
      exit(0);
      }

   return MilDisplay3D;
   }
