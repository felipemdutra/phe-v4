#include "../../include/util/util.h"

#include <wrapgl/vertex_layout.h>

static bool setup = false;

wgl::VertexLayout GetGlobalVertexLayout()
{
        static wgl::VertexLayout layout;
        static bool setup = false;

        if (!setup) {
                layout.AddAttr(0, wgl::AttributeType::kPosition, 3, GL_FLOAT, false);
                layout.AddAttr(1, wgl::AttributeType::kColor, 3, GL_FLOAT, true);
                setup = true;
        }

        return layout;
}

