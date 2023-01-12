#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

set(GENERATED_FILES
    azqtpyside_module_wrapper.cpp
    azqtcomponents_wrapper.cpp
                                                      
    azqtcomponents_buttondivider_wrapper.cpp
    azqtcomponents_buttonstripe_wrapper.cpp
    azqtcomponents_confighelpers_wrapper.cpp
    azqtcomponents_confighelpers_groupguard_wrapper.cpp
    azqtcomponents_dockbar_wrapper.cpp
    azqtcomponents_dockbarbutton_wrapper.cpp
    azqtcomponents_dockbarbutton_config_wrapper.cpp
    azqtcomponents_dockmainwindow_wrapper.cpp
    azqtcomponents_docktabbar_wrapper.cpp
    azqtcomponents_docktabwidget_wrapper.cpp
    azqtcomponents_extendedlabel_wrapper.cpp    
    azqtcomponents_fancydocking_widgetgrab_wrapper.cpp
    azqtcomponents_fancydocking_wrapper.cpp
    azqtcomponents_fancydockingdropzonewidget_wrapper.cpp
    azqtcomponents_fancydockingghostwidget_wrapper.cpp 
    
    azqtcomponents_filtercriteriabutton_wrapper.cpp
    azqtcomponents_searchtypefilter_wrapper.cpp
    azqtcomponents_searchtypeselector_wrapper.cpp
    azqtcomponents_filteredsearchwidget_wrapper.cpp
    azqtcomponents_filteredsearchwidget_config_wrapper.cpp

    
    flowlayout_wrapper.cpp
    azqtcomponents_globaleventfilter_wrapper.cpp
    azqtcomponents_helpbutton_wrapper.cpp
    azqtcomponents_interactivewindowgeometrychanger_wrapper.cpp
    azqtcomponents_searchlineedit_wrapper.cpp  
    azqtcomponents_style_wrapper.cpp
    azqtcomponents_styledbusylabel_wrapper.cpp
    azqtcomponents_styleddetailstablemodel_wrapper.cpp
    azqtcomponents_styleddetailstableview_wrapper.cpp
    azqtcomponents_styleddialog_wrapper.cpp
    azqtcomponents_styleddockwidget_wrapper.cpp
    azqtcomponents_styledlineedit_wrapper.cpp
    azqtcomponents_styleddoublespinbox_wrapper.cpp
    azqtcomponents_styledspinbox_wrapper.cpp  
    azqtcomponents_stylemanager_wrapper.cpp
    azqtcomponents_stylesheetcache_wrapper.cpp
    azqtcomponents_stylesheetpreprocessor_wrapper.cpp
    azqtcomponents_tagselector_wrapper.cpp
    
    azqtcomponents_titlebar_wrapper.cpp
    azqtcomponents_titlebar_config_wrapper.cpp
    azqtcomponents_titlebar_config_titlebar_wrapper.cpp
    azqtcomponents_titlebar_config_icon_wrapper.cpp
    azqtcomponents_titlebar_config_title_wrapper.cpp
    azqtcomponents_titlebar_config_buttons_wrapper.cpp
    
    azqtcomponents_titlebaroverdrawhandler_wrapper.cpp
#    azqtcomponents_toastnotification_wrapper.cpp
#    azqtcomponents_toastconfiguration_wrapper.cpp  
    azqtcomponents_toolbararea_wrapper.cpp
    azqtcomponents_toolbuttoncombobox_wrapper.cpp
    azqtcomponents_toolbuttonlineedit_wrapper.cpp
    azqtcomponents_toolbuttonwithwidget_wrapper.cpp
    azqtcomponents_vectoredit_wrapper.cpp
    azqtcomponents_windowdecorationwrapper_wrapper.cpp   
    
 #   azqtcomponents_assetfolderlistview_wrapper.cpp
    azqtcomponents_assetfolderthumbnailview_wrapper.cpp
    azqtcomponents_assetfolderthumbnailview_config_wrapper.cpp
    azqtcomponents_assetfolderthumbnailview_config_thumbnail_wrapper.cpp
    azqtcomponents_assetfolderthumbnailview_config_expandbutton_wrapper.cpp
    azqtcomponents_assetfolderthumbnailview_config_childframe_wrapper.cpp
    azqtcomponents_breadcrumbs_wrapper.cpp
    azqtcomponents_breadcrumbs_config_wrapper.cpp
    azqtcomponents_browseedit_wrapper.cpp
    azqtcomponents_card_wrapper.cpp
    azqtcomponents_card_config_wrapper.cpp
    azqtcomponents_cardheader_wrapper.cpp
    azqtcomponents_cardnotification_wrapper.cpp
    azqtcomponents_checkbox_wrapper.cpp
    azqtcomponents_checkbox_config_wrapper.cpp
 #   azqtcomponents_colorlabel_wrapper.cpp
    azqtcomponents_comboboxvalidator_wrapper.cpp
    azqtcomponents_combobox_wrapper.cpp
    azqtcomponents_combobox_config_wrapper.cpp
    azqtcomponents_dialogbuttonbox_wrapper.cpp
    
    azqtcomponents_draganddrop_wrapper.cpp
    azqtcomponents_draganddrop_dropindicator_wrapper.cpp
    azqtcomponents_draganddrop_dragindicator_wrapper.cpp
    azqtcomponents_draganddrop_config_wrapper.cpp
    
    azqtcomponents_elidinglabel_wrapper.cpp
    azqtcomponents_eyedropper_wrapper.cpp
    azqtcomponents_eyedropper_config_wrapper.cpp
    azqtcomponents_filedialog_wrapper.cpp
 #   azqtcomponents_gradientslider_wrapper.cpp
    azqtcomponents_lineedit_wrapper.cpp
    azqtcomponents_lineedit_config_wrapper.cpp
    
    azqtcomponents_menu_wrapper.cpp
    azqtcomponents_menu_margins_wrapper.cpp
    azqtcomponents_menu_config_wrapper.cpp
    
    azqtcomponents_azmessagebox_wrapper.cpp
 #   azqtcomponents_overlaywidgetbutton_wrapper.cpp
 #   azqtcomponents_overlaywidget_wrapper.cpp
    azqtcomponents_progressbar_wrapper.cpp
    azqtcomponents_progressbar_config_wrapper.cpp
    
    azqtcomponents_pushbutton_wrapper.cpp
    azqtcomponents_pushbutton_gradient_wrapper.cpp
    azqtcomponents_pushbutton_colorset_wrapper.cpp
    azqtcomponents_pushbutton_border_wrapper.cpp
    azqtcomponents_pushbutton_frame_wrapper.cpp
    azqtcomponents_pushbutton_smallicon_wrapper.cpp
    azqtcomponents_pushbutton_iconbutton_wrapper.cpp
    azqtcomponents_pushbutton_dropdownbutton_wrapper.cpp
    azqtcomponents_pushbutton_config_wrapper.cpp
    
    azqtcomponents_radiobutton_wrapper.cpp
    azqtcomponents_radiobutton_config_wrapper.cpp
    azqtcomponents_scrollbar_wrapper.cpp
    azqtcomponents_scrollbar_config_wrapper.cpp
    azqtcomponents_segmentbar_wrapper.cpp
    azqtcomponents_segmentcontrol_wrapper.cpp
    
    azqtcomponents_slider_wrapper.cpp
    azqtcomponents_slider_border_wrapper.cpp
    azqtcomponents_slider_gradientsliderconfig_wrapper.cpp
    azqtcomponents_slider_sliderconfig_wrapper.cpp
    azqtcomponents_slider_sliderconfig_handleconfig_wrapper.cpp
    azqtcomponents_slider_sliderconfig_grooveconfig_wrapper.cpp
    azqtcomponents_slider_config_wrapper.cpp
    azqtcomponents_sliderint_wrapper.cpp
    azqtcomponents_sliderdouble_wrapper.cpp
    azqtcomponents_slidercombo_wrapper.cpp
    azqtcomponents_sliderdoublecombo_wrapper.cpp
    
    azqtcomponents_spinbox_wrapper.cpp
    azqtcomponents_spinbox_config_wrapper.cpp
    azqtcomponents_statusbar_wrapper.cpp
    azqtcomponents_statusbar_config_wrapper.cpp
 #   azqtcomponents_tableview_wrapper.cpp
 #   azqtcomponents_tableview_config_wrapper.cpp
  #  azqtcomponents_tableviewmodel_wrapper.cpp
 #   azqtcomponents_tableviewitemdelegate_wrapper.cpp
    azqtcomponents_tabwidgetactiontoolbarcontainer_wrapper.cpp
    azqtcomponents_tabwidget_wrapper.cpp
    azqtcomponents_tabwidget_config_wrapper.cpp
    azqtcomponents_tabwidgetactiontoolbar_wrapper.cpp
    azqtcomponents_tabbar_wrapper.cpp
    
    azqtcomponents_text_wrapper.cpp
    azqtcomponents_text_config_wrapper.cpp
    azqtcomponents_toolbar_wrapper.cpp
    azqtcomponents_toolbar_toolbarconfig_wrapper.cpp
    azqtcomponents_toolbar_config_wrapper.cpp
    
    azqtcomponents_toolbutton_wrapper.cpp
    azqtcomponents_toolbutton_config_wrapper.cpp
 #   azqtcomponents_treeview_wrapper.cpp
 #   azqtcomponents_treeview_config_wrapper.cpp
 #   azqtcomponents_branchdelegate_wrapper.cpp
 #   azqtcomponents_styledtreeview_wrapper.cpp
 #   azqtcomponents_styledtreewidget_wrapper.cpp
    azqtcomponents_vectoreditelement_wrapper.cpp
    azqtcomponents_vectorelement_wrapper.cpp
    azqtcomponents_vectorinput_wrapper.cpp
    
    azqtcomponents_internal_wrapper.cpp
)
