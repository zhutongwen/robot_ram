/*

*/


#include <iostream>
#include <string>
#include "tinyxml2.h"
#include <unistd.h>
#include <libgen.h>

using namespace std;
using namespace tinyxml2;
void example1()
{
   XMLDocument doc;
//   if(doc.LoadFile("/home/z/work/ecat_master/resource/test.xml") != XML_SUCCESS)
   if(doc.LoadFile("./test.xml") != XML_SUCCESS)
   {
       cout << "load text.xml file fault!" << doc.LoadFile("test.xml") << endl;
       exit(1);
   }

   XMLElement *root=doc.RootElement();
   XMLElement *surface=root->FirstChildElement("node");
   while (surface)
   {
       XMLElement *surfaceChild=surface->FirstChildElement();
       const char* content;
       const XMLAttribute *attributeOfSurface = surface->FirstAttribute();
       cout<< attributeOfSurface->Name() << ":" << attributeOfSurface->Value() << endl;
       while(surfaceChild)
       {
           content=surfaceChild->GetText();
           surfaceChild=surfaceChild->NextSiblingElement();
           cout<<content<<endl;
       }
       surface=surface->NextSiblingElement();
   }
}
int main(int argc, char** argv)
{
    chdir(dirname(argv[0]));
   example1();
   return 0;
}
