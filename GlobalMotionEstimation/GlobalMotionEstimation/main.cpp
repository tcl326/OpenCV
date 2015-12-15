CvPoint2D32f p0, p1;
vector<Point2f,allocator<Point2f>> ax, by;
ax.push_back(Point2f(2,2));
by.push_back(Point2f(3,2));
Mat t = estimateGlobalMotionLeastSquares(ax,by,AFFINE,0);