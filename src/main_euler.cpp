/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob, Romain Prévost

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.x

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/block.h>
#include <nori/gui.h>
#include <filesystem/path.h>
#include <iomanip>

int main(int argc, char **argv) {
    using namespace nori;

    // If we have no argument or more than one argument ==> We can print it and stop the program
    if(argc != 2) {
        cerr << "Error while parsing the program arguments. You should only have one argument." << endl;
        return 0;
    }

    // In this case we are rendering on the euler cluster and we have no path to it
    // Open the UI with a dummy image
    ImageBlock block(Vector2i(720, 720), nullptr);
    RenderThread m_renderThread(block);
    std::string filename = argv[1];
    filesystem::path path(filename);

    const unsigned int FLOAT_PRECISION_OUTPUT = 2;
    const unsigned int SECONDS_SLEEP = 1;

    if (path.extension() == "xml") {
        // Render the XML scene file 
        m_renderThread.renderScene(filename);
        // Wait until the thread is done
        sleep(SECONDS_SLEEP);
        cout << std::setprecision(FLOAT_PRECISION_OUTPUT) << std::fixed << endl;
        while(!m_renderThread.isRenderingDone()) {
            float progress = m_renderThread.getProgressForEuler() * 100 ; 
            cout << "Progress of the rendering : " << progress << "%" << endl;
            sleep(SECONDS_SLEEP);   
        }
        cout << "Rendering done" << endl;
    }

    return 0;
}
