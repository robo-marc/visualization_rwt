var RwtUtils = {

    getTreeSpacer: function (indentLevel) {
        return '<span style="display:inline-block;height:1px;width:' + (15 * indentLevel) + 'px"></span>';
    },

    getTreeExpandButton: function () {
        return '<span class="toggle-wrap"><span class="toggle expand"></span></span>';
    },

    getTreeCollapseButton: function () {
        return '<span class="toggle-wrap"><span class="toggle collapse"></span></span>';
    },

    getTreeLeafButton: function () {
        return '<span class="toggle leaf"></span>'
    },

    isTreeToggleButton: function ($target) {
        return $target.hasClass('toggle') || $target.hasClass('toggle-wrap');
    },

}
