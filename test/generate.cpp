#ifndef GENERATE
#define GENERATE

#include <string>
#include <ctype.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <algorithm>
#include "query_tree.h"

using namespace std;

pair<int, int> dir[17] =
    {
		make_pair(-3, 0),
        make_pair(-3, 1),
        make_pair(-2, 2),
        make_pair(-1, 3),
        make_pair(0, 3),
        make_pair(1, 3),
        make_pair(2, 2),
        make_pair(3, 1),
        make_pair(3, 0),
        make_pair(3, -1),
        make_pair(2, -2),
        make_pair(1, -3),
        make_pair(0, -3),
        make_pair(-1, -3),
        make_pair(-2, -2),
        make_pair(-3, -1),
        make_pair(0, 0)};

char* bibtex = {};

// "If you use this in published work, please cite:\n"
// "  Fusing Points and Lines for High Performance Tracking, E. Rosten and T. Drummond, ICCV 2005\n"
// "  Machine learning for high-speed corner detection, E. Rosten and T. Drummond, ECCV 2006\n"
// "The Bibtex entries are:\n"
// "\n"
// "@inproceedings{rosten_2005_tracking,\n"
// "    title       =    \"Fusing points and lines for high performance tracking.\",\n"
// "    author      =    \"Edward Rosten and Tom Drummond\",\n"
// "    year        =    \"2005\",\n"
// "    month       =    \"October\",\n"
// "    pages       =    \"1508--1511\",\n"
// "    volume      =    \"2\",\n"
// "    booktitle   =    \"IEEE International Conference on Computer Vision\",\n"
// "    notes       =    \"Oral presentation\",\n"
// "    url         =    \"http://mi.eng.cam.ac.uk/~er258/work/rosten_2005_tracking.pdf\"\n"
// "}\n"
// "\n"
// "@inproceedings{rosten_2006_machine,\n"
// "    title       =    \"Machine learning for high-speed corner detection\",\n"
// "    author      =    \"Edward Rosten and Tom Drummond\",\n"
// "    year        =    \"2006\",\n"
// "    month       =    \"May\",\n"
// "    booktitle   =    \"European Conference on Computer Vision (to appear)\",\n"
// "    notes       =    \"Poster presentation\",\n"
// "    url         =    \"http://mi.eng.cam.ac.uk/~er258/work/rosten_2006_machine.pdf\"\n"
// "}\n";

class print_code
{
	public:
		enum WhichTest
		{
			Positive, Negative, Both
		};

		virtual void print_if(const std::string& indent, const std::string& test) const=0;
		virtual void print_elseif(const std::string& indent, const std::string& test) const=0;
		virtual void print_else(const std::string& indent) const=0;
		virtual void print_endif(const std::string& indent) const=0;
		virtual void print_continue(const std::string& indent) const=0;
		virtual void print_success(const std::string& indent) const=0;
		virtual std::string pos_test(int pixel) const=0;
		virtual std::string neg_test(int pixel) const=0;
		virtual std::string both_tests(int pixel) const=0;
		virtual ~print_code(){}
		
		print_code(const std::vector<query_tree>& tree);


	protected:
		void visit_tree(int node, std::string indent) const;
		const std::vector<query_tree>& tree;
};



string itoa(int i)
{
	ostringstream o;
	o << i;
	return o.str();
}

//Stringify tree so that trees can be compared with ==
string tree_to_string(const vector<query_tree>&tree, int node)
{
	if(node == query_tree::is_not_a_feature)
		return "c";
	else if(node == query_tree::is_a_feature)
		return "g";
	else
		return "p"+itoa(tree[node].pixel)+tree_to_string(tree, tree[node].positive)+tree_to_string(tree, tree[node].negative)+tree_to_string(tree, tree[node].neither)+"";
}

print_code::print_code(const vector<query_tree>& qtree)
:tree(qtree)
{
}

void print_code::visit_tree(int node, string indent) const
{
	indent = indent + "    ";

	if(node == query_tree::is_not_a_feature)
		print_continue(indent);
	else if(node == query_tree::is_a_feature)
		print_success(indent);
	else
	{
		//Subtree splatting
		//Get a signature for each subtree so we can see if they are the same
		//Default values are not the same (incase splat_subtree is off)
		string positive_tree="1";
		string negative_tree="2";
		string neither_tree="3";


		int pixel = tree[node].pixel;
		int positive = tree[node].positive;
		int negative = tree[node].negative;
		int neither = tree[node].neither;

		


			positive_tree = tree_to_string(tree,positive);
			negative_tree = tree_to_string(tree,negative);
			neither_tree  = tree_to_string(tree,neither);
		
		
		//Now we have stringified trees, we can compare them for equality :-)
		//There are several combinations, all the same (1), any pair the same (3) and none the same (1), giving 5 in total	
		if((negative_tree == neither_tree) && (positive_tree == neither_tree))
		{
			//This souldn't happen. I don't think. Well, maybe.
			//Maybe:

			//       - ? +
			//     -       + 
			//   -           +
			//   -           +
			//
			//Testing ? might not affect the final outcome if there is incomplete coverage.
			//
			//
			visit_tree(tree[node].neither, indent);
		}
		else if(negative_tree == neither_tree)
		{
			//Do positive test only
			print_if(indent, pos_test(pixel));
				visit_tree(positive, indent);
			print_else(indent);
				visit_tree(neither, indent);
			print_endif(indent);
		}
		else if(positive_tree == neither_tree)
		{
			//Do negative test only
			print_if(indent, neg_test(pixel));
				visit_tree(negative, indent);
			print_else(indent);
				visit_tree(neither, indent);
			print_endif(indent);
		}
		else if(negative_tree == positive_tree)
		{
			//I can't conceive why this should happen!
			//Maybe if there is insufficient coverage of the space...?
			//Yes, it can happen, I suppose.

			print_if(indent, both_tests(pixel));
				visit_tree(positive, indent);
			print_else(indent);
				visit_tree(neither, indent);
			print_endif(indent);
		}
		else
		{
			print_if(indent, pos_test(pixel));
				visit_tree(positive, indent);
			print_elseif(indent, neg_test(pixel));
				visit_tree(negative, indent);
			print_else(indent);
				visit_tree(neither, indent);
			print_endif(indent);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class print_c:public print_code
{
	public:
		virtual void print_if(const string & indent, const string& test) const
		{
			cout << indent << "if(" << test << ")" << endl;
		}

		virtual void print_elseif(const string & indent, const string& test) const
		{
			cout << indent << "else if(" << test << ")" << endl;
		}

		virtual void print_else(const string& indent) const
		{
			cout << indent << "else" << endl;
		}

		virtual void print_endif(const string& indent) const
		{
		}

		virtual void print_continue(const string& indent) const
		{
			cout << indent << "continue;" << endl;
		}
		
		virtual void print_success(const string& indent) const
		{
			cout << indent << "goto success;" << endl;
		}

		virtual string pos_test(int pixel) const
		{
			return pixel_access[pixel] + " > cb";
		}

		virtual string neg_test(int pixel) const
		{
			return pixel_access[pixel] + " < c_b";
		}

		virtual string both_tests(int pixel) const
		{
			return pos_test(pixel) + "||" + neg_test(pixel);
		}

		print_c(const vector<query_tree>& qtree, bool use_cpp)
		:print_code(qtree)
		{
			generate_pixel_accessor();
			if(use_cpp)
				generate_cpp();
			else
				generate_c();
		}

	private:
		vector<string> pixel_access;
		vector<int> caches;

		//This function figures out which rows should have pointers to them
		//cached and automatically incremented.
		void generate_pixel_accessor();
		void generate_c() const;
		void generate_cpp() const;
};

void print_c::generate_pixel_accessor()
{   

	//First, compute the frequency of access.
	vector<pair<int,int> > counts(16);
	
	//Number of things to cache
	unsigned int cache_pointers = 0;
	//The first one is a bodge because we know we're visiting the centre point.
	

	for(int i=0; i < 16; i++)
		counts[i].second = i;
	
	for(unsigned int i=0; i < tree.size(); i++)
		counts[tree[i].pixel].first += tree[i].num_of_tests;

	sort(counts.begin(), counts.end());
	reverse(counts.begin(), counts.end());

	pixel_access.resize(16);

	//Hack to put in cache for the centre point.
	caches.push_back(17);
	
	//Figure out how to access each pixel
	//ie is it a big index, or can it use cached
	//row pointers directly or indirectly
	for(int i=0; i < 16; i++)
	{
		int crnt = counts[i].second;
		
		for(unsigned int j=0; j < caches.size(); j++)
			if(dir[crnt].first == dir[caches[j]].first)
			{
				//There's already a cached pixel on this row.
				// pixel_access[crnt] = "*(cache_"+itoa(j)+"+" + itoa(dir[crnt].second-dir[caches[j]].second) + ")";
                pixel_access[crnt] = "im.at<int>(row + " + itoa(dir[j].first) + ", col + " + itoa(dir[j].second) + ")";
				goto done;
			}

		if(caches.size() < cache_pointers)
		{
			pixel_access[crnt] = "*cache_" + itoa(caches.size());
			caches.push_back(crnt);
		}
		else
		{
			pixel_access[crnt] = "im.at<int>(row + " + itoa(dir[crnt].first) + ", col + " + itoa(dir[crnt].second) + ")";
		}

		done:
		;

	}
}



void print_c::generate_c() const
{
	// gvar3<bool>   opencv("output_language.c.opencv", false, 1);
	// gvar3<bool>   stridein("output_language.c.stride_input", false, 1);

	//Start to output code
	cout <<
"#include <stdlib.h>																			\n";

	if(false)
		cout <<
"#include <cv.h>																				\n"
"typedef CvPoint xy;																			\n"
"typedef unsigned char byte;																	\n";
	else
		cout << 
"#include \"fast.h\"																			\n";



	if(false)
		cout <<
"xy* " << "fast_corner_detect" << "(const byte* im, int xsize, int xstride, int ysize, int barrier, int* num)	\n"
"{																								\n";
	else
		cout << 
"xy* " << "fast_corner_detect"<< "(const byte* im, int xsize, int ysize, int barrier, int* num)				\n"
"{																								\n"
"	int xstride = xsize;																		\n";

	
	cout << 
"	int boundary = 3, y, cb, c_b;																\n"
"	const byte  *line_max, *line_min;															\n"
"	int			rsize=512, total=0;																\n"
"	xy	 		*ret = (xy*)malloc(rsize*sizeof(xy));											\n";

	for(unsigned int i=0; i < caches.size(); i++)
		cout << "	const byte* cache_"+itoa(i)+";\n";
cout <<
"	int	pixel[16];																				\n";
	for(int i=0; i < 16; i++)
		cout << "	pixel[" << i << "] = " << dir[i].second << " + " << dir[i].first << " * xstride;		\n";

cout <<
"	for(y = boundary ; y < ysize - boundary; y++)												\n"
"	{																							\n"
"		cache_0 = im + boundary + y*xstride;													\n"
"		line_min = cache_0 - boundary;															\n"
"		line_max = im + xsize - boundary + y * xsize;											\n"
"																								\n";
	//Cache_0 is a special case.
	for(unsigned int i=1; i < caches.size(); i++)
		cout << "		cache_"+itoa(i)+" = cache_0 + pixel[" + itoa(caches[i]) + "];\n";

cout <<
"																								\n"
"		for(; cache_0 < line_max;";

	for(unsigned int i=0; i < caches.size(); i++)
		cout << "cache_"+itoa(i)+"++" + (i==caches.size()-1?"":", ");
	cout << ")\n"
"		{																						\n"
"			cb = *cache_0 + barrier;															\n"
"			c_b = *cache_0 - barrier;															\n";
			visit_tree(0, "        ");
cout <<
"			success:																			\n"
"				if(total >= rsize)																\n"
"				{																				\n"
"					rsize *=2;																	\n"
"					ret=(xy*)realloc(ret, rsize*sizeof(xy));									\n"
"					if(ret == NULL)																\n"
"					{																			\n"
"						*num=-1;																\n"
"						return NULL;															\n"
"					}																			\n"
"				}																				\n"
"				ret[total].x = cache_0-line_min;												\n"
"				ret[total++].y = y;																\n"
"		}																						\n"	
"	}																							\n"	
"	*num = total;																				\n"
"	return ret;																					\n"
"}																								\n"
"																								\n";
}



void print_c::generate_cpp() const
{
	//Start to output code
	cout <<

"#include <vector>																				\n"
"#include <opencv2/core/core.hpp>																				\n"
"#include <opencv2/highgui/highgui.hpp>																				\n"
"#include <opencv2/features2d.hpp>																				\n"
"#include <opencv2/imgproc.hpp>																				\n"
"																								\n"

"using namespace std;																			\n"
"																								\n"
"pair<int, int> dir[17] ={make_pair(0, 3),make_pair(1, 3),make_pair(2, 2),make_pair(3, 1),make_pair(3, 0),make_pair(3, -1),make_pair(2, -2),make_pair(1, -3),make_pair(0, -3),make_pair(-1, -3),make_pair(-2, -2),make_pair(-3, -1),make_pair(-3, 0),make_pair(-3, 1),make_pair(-2, 2),make_pair(-1, 3),make_pair(0, 0)};																																							\n"
"																								\n"
"																								\n"
"void " << "fast_corner_detect" << "(const cv::Mat &im, vector<pair<int, int>>& corners, int barrier)		\n"
"{																								\n"

	// for(unsigned int i=0; i < caches.size(); i++)
	// 	cout << "	const int* cache_"+itoa(i)+";\n";

"																								\n"
"	for(int row = 3 ; row < im.rows - 3; row++)											\n"
"	{																							\n"

"																								\n";
	//Cache_0 is a special case.
	// for(unsigned int i=1; i < caches.size(); i++)
	// 	cout << "		cache_"+itoa(i)+" = cache_0 + pixel[" + itoa(caches[i]) + "];\n";

cout <<
"																								\n"
"		for(int col = 3; col < im.cols;col++)";

	// for(unsigned int i=0; i < caches.size(); i++)
	// 	cout << "cache_"+itoa(i)+"++" + (i==caches.size()-1?"":", ");
	cout << 
"		{																						\n"
"			int cb = im.at<int>(row, col) + barrier;															\n"
"			int c_b = im.at<int>(row, col) - barrier;															\n";
			visit_tree(0, "        ");
cout <<
"			success:																			\n"
"				corners.push_back(make_pair(row, col));								\n"
"		}																						\n"	
"	}																							\n"	
"}																								\n"
"																								\n";
cout << endl;
cout << endl;
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


void generate_cpp(const vector<query_tree>& t, string comments)
{
	cout << "/*" << comments << endl << endl << "*/\n";
	print_c a(t, 1);
}

void generate_c(const vector<query_tree>&t, string comments)
{
	cout << "/*" << comments  << endl <<  endl << "*/\n";
	print_c a(t, 0);
}

#endif