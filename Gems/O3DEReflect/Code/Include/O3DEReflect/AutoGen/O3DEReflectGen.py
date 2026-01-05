#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

"""
O3DEReflect Code Generator

This module extends the AzAutoGen system to provide Unreal-style reflection
code generation for O3DE components, structs, and enums.

Usage:
    python O3DEReflectGen.py --input MyComponent.O3DEReflect.xml --output-dir ./Generated
"""

import os
import re
import sys
import json
import argparse
import hashlib
import logging
import glob
from pathlib import Path
from xml.etree import ElementTree as ET

# Add cmake directory to path for accessing AzAutoGen utilities
script_dir = os.path.dirname(os.path.abspath(__file__))
cmake_dir = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', '..', 'cmake'))
if cmake_dir not in sys.path:
    sys.path.insert(0, cmake_dir)

# Try to find O3DE's Python venv site-packages for Jinja2
def find_o3de_site_packages():
    """Find O3DE's Python venv site-packages directory."""
    # Common locations for O3DE Python venv
    home = os.path.expanduser("~")
    patterns = [
        os.path.join(home, ".o3de", "Python", "venv", "*", "lib", "site-packages"),
        os.path.join(home, ".o3de", "Python", "venv", "*", "Lib", "site-packages"),  # Windows
    ]
    for pattern in patterns:
        matches = glob.glob(pattern)
        if matches:
            return matches[0]
    return None

# Add O3DE site-packages to path if needed
o3de_site_packages = find_o3de_site_packages()
if o3de_site_packages and o3de_site_packages not in sys.path:
    sys.path.insert(0, o3de_site_packages)

try:
    from jinja2 import Environment, FileSystemLoader, select_autoescape
    JINJA2_AVAILABLE = True
except ImportError:
    JINJA2_AVAILABLE = False
    print("Warning: Jinja2 not available. Template rendering will fail.", file=sys.stderr)
    if o3de_site_packages:
        print(f"  Tried O3DE site-packages at: {o3de_site_packages}", file=sys.stderr)

logging.basicConfig(format='[%(levelname)s] %(name)s: %(message)s')
logger = logging.getLogger('O3DEReflectGen')
logger.setLevel(logging.INFO)


def create_hash_guid(string: str) -> str:
    """
    Generate a deterministic UUID from a string using MD5 hash.
    Matches the CreateHashGuid function in AzAutoGen.py.
    """
    hash_obj = hashlib.md5(string.encode('utf-8'))
    hash_str = hash_obj.hexdigest()
    return "{{{}-{}-{}-{}-{}}}".format(
        hash_str[0:8].upper(),
        hash_str[8:12].upper(),
        hash_str[12:16].upper(),
        hash_str[16:20].upper(),
        hash_str[20:].upper()
    )


def boolean_true(value) -> bool:
    """Check if a string value represents true."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower().strip() in ('true', '1', 'yes')
    return bool(value)


def camel_to_human(string: str) -> str:
    """Convert camelCase to Human Readable format."""
    if not string:
        return string
    result = string[0].upper()
    for char in string[1:]:
        if char.isupper():
            result += ' '
        result += char
    return result


def sanitize_path(path: str) -> str:
    """Normalize path separators."""
    return path.replace('\\', '/')


class O3DEReflectParser:
    """Parser for O3DEReflect XML files."""
    
    def __init__(self, input_file: str):
        self.input_file = input_file
        self.tree = None
        self.root = None
        
    def parse(self) -> dict:
        """Parse the XML file and return a structured dictionary."""
        self.tree = ET.parse(self.input_file)
        self.root = self.tree.getroot()
        
        result = {
            'components': [],
            'structs': [],
            'enums': [],
            'source_file': os.path.basename(self.input_file)
        }
        
        # Handle single element root (Component, Struct, or Enum)
        if self.root.tag in ('Component', 'Struct', 'Enum'):
            elements = [self.root]
        else:
            # Handle O3DEReflect wrapper with multiple elements
            elements = list(self.root)
        
        for element in elements:
            if element.tag == 'Component':
                result['components'].append(self._parse_component(element))
            elif element.tag == 'Struct':
                result['structs'].append(self._parse_struct(element))
            elif element.tag == 'Enum':
                result['enums'].append(self._parse_enum(element))
        
        return result
    
    def _parse_component(self, elem: ET.Element) -> dict:
        """Parse a Component element."""
        name = elem.attrib.get('Name', '')
        namespace = elem.attrib.get('Namespace', '')
        full_name = f"{namespace}::{name}" if namespace else name
        
        return {
            'name': name,
            'namespace': namespace,
            'full_name': full_name,
            'uuid': elem.attrib.get('Uuid') or create_hash_guid(full_name),
            'category': elem.attrib.get('Category', 'General'),
            'description': elem.attrib.get('Description', f'{name} component'),
            'display_name': elem.attrib.get('DisplayName', camel_to_human(name)),
            'icon': elem.attrib.get('Icon', ''),
            'hide_in_editor': boolean_true(elem.attrib.get('HideInEditor', 'false')),
            'abstract': boolean_true(elem.attrib.get('Abstract', 'false')),
            'menu_category': elem.attrib.get('AppearsInAddComponentMenu', 'Game'),
            'includes': self._parse_includes(elem),
            'base_classes': self._parse_base_classes(elem),
            'services': self._parse_services(elem),
            'properties': self._parse_properties(elem),
            'functions': self._parse_functions(elem),
        }
    
    def _parse_struct(self, elem: ET.Element) -> dict:
        """Parse a Struct element."""
        name = elem.attrib.get('Name', '')
        namespace = elem.attrib.get('Namespace', '')
        full_name = f"{namespace}::{name}" if namespace else name
        
        return {
            'name': name,
            'namespace': namespace,
            'full_name': full_name,
            'uuid': elem.attrib.get('Uuid') or create_hash_guid(full_name),
            'category': elem.attrib.get('Category', 'Data'),
            'description': elem.attrib.get('Description', f'{name} data structure'),
            'display_name': elem.attrib.get('DisplayName', camel_to_human(name)),
            'blueprint_type': boolean_true(elem.attrib.get('BlueprintType', 'true')),
            'atomic': boolean_true(elem.attrib.get('Atomic', 'false')),
            'includes': self._parse_includes(elem),
            'properties': self._parse_properties(elem),
        }
    
    def _parse_enum(self, elem: ET.Element) -> dict:
        """Parse an Enum element."""
        name = elem.attrib.get('Name', '')
        namespace = elem.attrib.get('Namespace', '')
        full_name = f"{namespace}::{name}" if namespace else name
        
        return {
            'name': name,
            'namespace': namespace,
            'full_name': full_name,
            'uuid': elem.attrib.get('Uuid') or create_hash_guid(full_name),
            'category': elem.attrib.get('Category', 'Enums'),
            'description': elem.attrib.get('Description', f'{name} enumeration'),
            'display_name': elem.attrib.get('DisplayName', camel_to_human(name)),
            'blueprint_type': boolean_true(elem.attrib.get('BlueprintType', 'true')),
            'is_flags': boolean_true(elem.attrib.get('Flags', 'false')),
            'underlying_type': elem.attrib.get('UnderlyingType', 'int32_t'),
            'values': self._parse_enum_values(elem),
        }
    
    def _parse_includes(self, elem: ET.Element) -> list:
        """Parse Include elements."""
        return [inc.attrib.get('File', '') for inc in elem.findall('Include')]
    
    def _parse_base_classes(self, elem: ET.Element) -> list:
        """Parse BaseClass elements."""
        result = []
        for base in elem.findall('BaseClass'):
            result.append({
                'name': base.attrib.get('Name', ''),
                'namespace': base.attrib.get('Namespace', ''),
                'include': base.attrib.get('Include', ''),
            })
        return result
    
    def _parse_services(self, elem: ET.Element) -> dict:
        """Parse Service elements grouped by type."""
        result = {
            'provides': [],
            'requires': [],
            'incompatible': [],
            'dependent': [],
        }
        for service in elem.findall('Service'):
            service_type = service.attrib.get('Type', '').lower()
            service_name = service.attrib.get('Name', '')
            if service_type in result:
                result[service_type].append(service_name)
        return result
    
    def _parse_properties(self, elem: ET.Element) -> list:
        """Parse Property elements."""
        result = []
        for prop in elem.findall('Property'):
            prop_name = prop.attrib.get('Name', '')
            result.append({
                'name': prop_name,
                'member_name': prop_name if prop_name.startswith('m_') else f'm_{prop_name}',
                'type': prop.attrib.get('Type', 'float'),
                'default': prop.attrib.get('Default', ''),
                'display_name': prop.attrib.get('DisplayName', camel_to_human(prop_name.lstrip('m_'))),
                'category': prop.attrib.get('Category', ''),
                'tooltip': prop.attrib.get('Tooltip', ''),
                'suffix': prop.attrib.get('Suffix', ''),
                # Editor visibility
                'edit_anywhere': boolean_true(prop.attrib.get('EditAnywhere', 'true')),
                'edit_defaults_only': boolean_true(prop.attrib.get('EditDefaultsOnly', 'false')),
                'edit_instance_only': boolean_true(prop.attrib.get('EditInstanceOnly', 'false')),
                'visible_anywhere': boolean_true(prop.attrib.get('VisibleAnywhere', 'false')),
                'read_only': boolean_true(prop.attrib.get('ReadOnly', 'false')),
                # Script visibility
                'blueprint_read_write': boolean_true(prop.attrib.get('BlueprintReadWrite', 'true')),
                'blueprint_read_only': boolean_true(prop.attrib.get('BlueprintReadOnly', 'false')),
                'expose_to_script': boolean_true(prop.attrib.get('ExposeToScript', 'true')),
                # Numeric constraints
                'min': prop.attrib.get('Min'),
                'max': prop.attrib.get('Max'),
                'ui_min': prop.attrib.get('UIMin'),
                'ui_max': prop.attrib.get('UIMax'),
                'step': prop.attrib.get('Step'),
                # Advanced
                'change_notify': prop.attrib.get('ChangeNotify', ''),
                'visibility': prop.attrib.get('Visibility', ''),
                'order': int(prop.attrib.get('Order', '0')),
            })
        return result
    
    def _parse_functions(self, elem: ET.Element) -> list:
        """Parse Function elements."""
        result = []
        for func in elem.findall('Function'):
            func_name = func.attrib.get('Name', '')
            return_elem = func.find('Return')
            params = []
            for param in func.findall('Param'):
                params.append({
                    'name': param.attrib.get('Name', ''),
                    'type': param.attrib.get('Type', ''),
                    'default': param.attrib.get('Default', ''),
                    'display_name': param.attrib.get('DisplayName', ''),
                    'tooltip': param.attrib.get('Tooltip', ''),
                })
            
            result.append({
                'name': func_name,
                'display_name': func.attrib.get('DisplayName', camel_to_human(func_name)),
                'category': func.attrib.get('Category', ''),
                'tooltip': func.attrib.get('Tooltip', ''),
                'return_type': return_elem.attrib.get('Type', 'void') if return_elem is not None else 'void',
                'parameters': params,
                # Flags
                'blueprint_callable': boolean_true(func.attrib.get('BlueprintCallable', 'true')),
                'blueprint_pure': boolean_true(func.attrib.get('BlueprintPure', 'false')),
                'call_in_editor': boolean_true(func.attrib.get('CallInEditor', 'false')),
                'server': boolean_true(func.attrib.get('Server', 'false')),
                'client': boolean_true(func.attrib.get('Client', 'false')),
                'net_multicast': boolean_true(func.attrib.get('NetMulticast', 'false')),
            })
        return result
    
    def _parse_enum_values(self, elem: ET.Element) -> list:
        """Parse EnumValue elements."""
        result = []
        for val in elem.findall('EnumValue'):
            val_name = val.attrib.get('Name', '')
            result.append({
                'name': val_name,
                'value': val.attrib.get('Value'),
                'display_name': val.attrib.get('DisplayName', camel_to_human(val_name)),
                'tooltip': val.attrib.get('Tooltip', ''),
                'hidden': boolean_true(val.attrib.get('Hidden', 'false')),
            })
        return result


class O3DEReflectGenerator:
    """Code generator using Jinja2 templates."""
    
    def __init__(self, template_dir: str, output_dir: str):
        self.template_dir = template_dir
        self.output_dir = output_dir
        
        if not JINJA2_AVAILABLE:
            raise RuntimeError("Jinja2 is required for code generation")
        
        self.env = Environment(
            loader=FileSystemLoader(template_dir),
            autoescape=select_autoescape(['html', 'xml']),
            trim_blocks=True,
            lstrip_blocks=True,
        )
        
        # Add custom filters
        self.env.filters['booleanTrue'] = boolean_true
        self.env.globals['CreateHashGuid'] = create_hash_guid
        self.env.globals['CamelToHuman'] = camel_to_human
    
    def generate(self, parsed_data: dict, source_file: str):
        """Generate code from parsed data."""
        os.makedirs(self.output_dir, exist_ok=True)
        
        file_prefix = os.path.splitext(os.path.basename(source_file))[0]
        if file_prefix.endswith('.O3DEReflect'):
            file_prefix = file_prefix[:-12]
        
        # Get user code if available
        user_code = parsed_data.get('user_code', {})
        
        generated_files = []
        
        # Generate component files
        for component in parsed_data.get('components', []):
            header_file = self._generate_component_header(component, file_prefix, source_file)
            source_file_out = self._generate_component_source(component, file_prefix, source_file, user_code)
            generated_files.extend([header_file, source_file_out])
        
        # Generate struct files
        for struct in parsed_data.get('structs', []):
            header_file = self._generate_struct_header(struct, file_prefix, source_file)
            source_file_out = self._generate_struct_source(struct, file_prefix, source_file)
            generated_files.extend([header_file, source_file_out])
        
        # Generate enum files
        for enum in parsed_data.get('enums', []):
            header_file = self._generate_enum_header(enum, file_prefix, source_file)
            source_file_out = self._generate_enum_source(enum, file_prefix, source_file)
            generated_files.extend([header_file, source_file_out])
        
        return generated_files
    
    def _generate_component_header(self, component: dict, file_prefix: str, source_file: str) -> str:
        """Generate component header file."""
        output_file = os.path.join(self.output_dir, f"{component['name']}.AutoReflect.h")
        content = self._render_component_header(component, file_prefix, source_file)
        self._write_file(output_file, content)
        return output_file
    
    def _generate_component_source(self, component: dict, file_prefix: str, source_file: str, user_code: dict = None) -> str:
        """Generate component source file."""
        output_file = os.path.join(self.output_dir, f"{component['name']}.AutoReflect.cpp")
        content = self._render_component_source(component, file_prefix, source_file, user_code or {})
        self._write_file(output_file, content)
        return output_file
    
    def _generate_struct_header(self, struct: dict, file_prefix: str, source_file: str) -> str:
        """Generate struct header file."""
        output_file = os.path.join(self.output_dir, f"{struct['name']}.AutoReflect.h")
        content = self._render_struct_header(struct, file_prefix, source_file)
        self._write_file(output_file, content)
        return output_file
    
    def _generate_struct_source(self, struct: dict, file_prefix: str, source_file: str) -> str:
        """Generate struct source file."""
        output_file = os.path.join(self.output_dir, f"{struct['name']}.AutoReflect.cpp")
        content = self._render_struct_source(struct, file_prefix, source_file)
        self._write_file(output_file, content)
        return output_file
    
    def _generate_enum_header(self, enum: dict, file_prefix: str, source_file: str) -> str:
        """Generate enum header file."""
        output_file = os.path.join(self.output_dir, f"{enum['name']}.AutoReflect.h")
        content = self._render_enum_header(enum, file_prefix, source_file)
        self._write_file(output_file, content)
        return output_file
    
    def _generate_enum_source(self, enum: dict, file_prefix: str, source_file: str) -> str:
        """Generate enum source file."""
        output_file = os.path.join(self.output_dir, f"{enum['name']}.AutoReflect.cpp")
        content = self._render_enum_source(enum, file_prefix, source_file)
        self._write_file(output_file, content)
        return output_file
    
    def _render_component_header(self, component: dict, file_prefix: str, source_file: str) -> str:
        """Render component header using inline template."""
        return f'''/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * AUTO-GENERATED FILE - DO NOT EDIT
 * Generated by O3DEReflect from {os.path.basename(source_file)}
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Memory/SystemAllocator.h>
{self._render_includes(component.get('includes', []))}
{self._render_base_class_includes(component.get('base_classes', []))}

{self._render_namespace_begin(component.get('namespace', ''))}
    //! {component.get('description', '')}
    class {component['name']}
{self._render_base_class_inheritance(component.get('base_classes', []))}
    {{
    public:
        AZ_COMPONENT_DECL({component['name']});
        
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        {component['name']}() = default;
        ~{component['name']}() override = default;

{self._render_property_accessors(component['name'], component.get('properties', []))}
    protected:
        // AZ::Component interface
        void Init() override;
        void Activate() override;
        void Deactivate() override;

{self._render_function_declarations(component.get('functions', []))}
    private:
        // Member variables
{self._render_property_members(component.get('properties', []))}
    }};
{self._render_namespace_end(component.get('namespace', ''))}
'''
    
    def _render_component_source(self, component: dict, file_prefix: str, source_file: str, user_code: dict = None) -> str:
        """Render component source using inline template, merging user implementations."""
        user_code = user_code or {}
        user_functions = user_code.get('functions', {})
        
        # Build base class string, filtering out bus handlers (they shouldn't be in serializeContext)
        base_class_parts = []
        for b in component.get('base_classes', []):
            full_name = f"{b.get('namespace', '')}::{b['name']}" if b.get('namespace') else b['name']
            # Filter out bus handlers - they aren't serializable base classes
            if full_name.endswith('::Handler') or 'BusHandler' in full_name or 'Bus::Handler' in full_name:
                continue
            base_class_parts.append(full_name)
        base_classes_str = ', '.join(base_class_parts) if base_class_parts else 'AZ::Component'
        
        # Get component name and namespace for function lookup
        comp_name = component['name']
        namespace = component.get('namespace', '')
        full_class_name = f"{namespace}::{comp_name}" if namespace else comp_name
        
        return f'''/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * AUTO-GENERATED FILE - DO NOT EDIT
 * Generated by O3DEReflect from {os.path.basename(source_file)}
 */

#include "{component['name']}.AutoReflect.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Script/ScriptContextAttributes.h>

{self._render_namespace_begin(component.get('namespace', ''))}

    AZ_COMPONENT_IMPL({component['name']}, "{component['name']}", "{component['uuid']}"{self._render_base_class_list(component.get('base_classes', []), exclude_bus_handlers=True)});

    void {component['name']}::Reflect(AZ::ReflectContext* context)
    {{
        // SerializeContext
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {{
            serializeContext->Class<{component['name']}, {base_classes_str}>()
                ->Version(1)
{self._render_serialize_fields(component['name'], component.get('properties', []))}
                ;

            // EditContext
            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {{
                editContext->Class<{component['name']}>("{component.get('display_name', component['name'])}", "{component.get('description', '')}")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "{component.get('category', 'General')}")
{self._render_edit_class_attributes(component)}
{self._render_edit_data_elements(component['name'], component.get('properties', []))}
                    ;
            }}
        }}

        // BehaviorContext
        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {{
            behaviorContext->Class<{component['name']}>("{component['name']}")
                ->Attribute(AZ::Script::Attributes::Category, "{component.get('category', 'General')}")
{self._render_behavior_properties(component['name'], component.get('properties', []))}
{self._render_behavior_methods(component['name'], component.get('functions', []))}
                ;
        }}
    }}

{self._render_service_functions(component['name'], component.get('services', {}))}

{self._render_user_implementations(comp_name, full_class_name, user_functions, component.get('functions', []))}

{self._render_namespace_end(component.get('namespace', ''))}
'''

    def _render_struct_header(self, struct: dict, file_prefix: str, source_file: str) -> str:
        """Render struct header."""
        return f'''/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * AUTO-GENERATED FILE - DO NOT EDIT
 * Generated by O3DEReflect from {os.path.basename(source_file)}
 */

#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Serialization/SerializeContext.h>
{self._render_includes(struct.get('includes', []))}

{self._render_namespace_begin(struct.get('namespace', ''))}
    //! {struct.get('description', '')}
    struct {struct['name']}
    {{
        AZ_TYPE_INFO({struct['name']}, "{struct['uuid']}");
        AZ_CLASS_ALLOCATOR({struct['name']}, AZ::SystemAllocator);

        static void Reflect(AZ::ReflectContext* context);

        {struct['name']}() = default;
        ~{struct['name']}() = default;

        bool operator==(const {struct['name']}& rhs) const;
        bool operator!=(const {struct['name']}& rhs) const {{ return !(*this == rhs); }}

        // Member variables
{self._render_struct_members(struct.get('properties', []))}
    }};
{self._render_namespace_end(struct.get('namespace', ''))}
'''

    def _render_struct_source(self, struct: dict, file_prefix: str, source_file: str) -> str:
        """Render struct source."""
        return f'''/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * AUTO-GENERATED FILE - DO NOT EDIT
 * Generated by O3DEReflect from {os.path.basename(source_file)}
 */

#include "{struct['name']}.AutoReflect.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/RTTI/BehaviorContext.h>

{self._render_namespace_begin(struct.get('namespace', ''))}

    void {struct['name']}::Reflect(AZ::ReflectContext* context)
    {{
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {{
            serializeContext->Class<{struct['name']}>()
                ->Version(1)
{self._render_serialize_fields(struct['name'], struct.get('properties', []))}
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {{
                editContext->Class<{struct['name']}>("{struct.get('display_name', struct['name'])}", "{struct.get('description', '')}")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "{struct.get('category', 'Data')}")
{self._render_edit_data_elements(struct['name'], struct.get('properties', []))}
                    ;
            }}
        }}

{self._render_struct_behavior_context(struct)}
    }}

    bool {struct['name']}::operator==(const {struct['name']}& rhs) const
    {{
{self._render_struct_equality(struct.get('properties', []))}
    }}

{self._render_namespace_end(struct.get('namespace', ''))}
'''

    def _render_enum_header(self, enum: dict, file_prefix: str, source_file: str) -> str:
        """Render enum header."""
        return f'''/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * AUTO-GENERATED FILE - DO NOT EDIT
 * Generated by O3DEReflect from {os.path.basename(source_file)}
 */

#pragma once

#include <AzCore/base.h>
#include <AzCore/RTTI/RTTI.h>

{self._render_namespace_begin(enum.get('namespace', ''))}
    //! {enum.get('description', '')}
    enum class {enum['name']} : {enum.get('underlying_type', 'int32_t')}
    {{
{self._render_enum_values(enum)}
    }};

{self._render_enum_bitwise_operators(enum)}
    // Enum reflection helper
    class {enum['name']}Reflect
    {{
    public:
        static void Reflect(AZ::ReflectContext* context);
    }};
{self._render_namespace_end(enum.get('namespace', ''))}
'''

    def _render_enum_source(self, enum: dict, file_prefix: str, source_file: str) -> str:
        """Render enum source."""
        return f'''/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * AUTO-GENERATED FILE - DO NOT EDIT
 * Generated by O3DEReflect from {os.path.basename(source_file)}
 */

#include "{enum['name']}.AutoReflect.h"

#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Script/ScriptContextAttributes.h>

{self._render_namespace_begin(enum.get('namespace', ''))}

    void {enum['name']}Reflect::Reflect(AZ::ReflectContext* context)
    {{
        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {{
{self._render_enum_behavior_values(enum)}
        }}
    }}

{self._render_namespace_end(enum.get('namespace', ''))}
'''

    # Helper methods for rendering
    def _render_includes(self, includes: list) -> str:
        return '\n'.join([f'#include <{inc}>' for inc in includes])
    
    def _render_base_class_includes(self, base_classes: list) -> str:
        includes = [b['include'] for b in base_classes if b.get('include')]
        return '\n'.join([f'#include <{inc}>' for inc in includes])
    
    def _render_base_class_inheritance(self, base_classes: list) -> str:
        if not base_classes:
            return '        : public AZ::Component'
        parts = []
        for b in base_classes:
            full_name = f"{b['namespace']}::{b['name']}" if b.get('namespace') else b['name']
            parts.append(f'public {full_name}')
        return '        : ' + ', '.join(parts)
    
    def _render_base_class_list(self, base_classes: list, exclude_bus_handlers: bool = False) -> str:
        """Render base class list for macros like AZ_COMPONENT_IMPL.
        
        Args:
            base_classes: List of base class dicts with 'name' and 'namespace' keys
            exclude_bus_handlers: If True, filter out bus handlers (end with ::Handler or BusHandler)
        """
        if not base_classes:
            return ''
        parts = []
        for b in base_classes:
            full_name = f"{b['namespace']}::{b['name']}" if b.get('namespace') else b['name']
            if exclude_bus_handlers:
                # Filter out bus handlers - they typically end with ::Handler or BusHandler
                if full_name.endswith('::Handler') or 'BusHandler' in full_name or 'Bus::Handler' in full_name:
                    continue
            parts.append(full_name)
        if not parts:
            return ''
        return ', ' + ', '.join(parts)
    
    def _render_namespace_begin(self, namespace: str) -> str:
        if not namespace:
            return ''
        return f'namespace {namespace}\n{{'
    
    def _render_namespace_end(self, namespace: str) -> str:
        if not namespace:
            return ''
        return f'}} // namespace {namespace}'
    
    def _render_property_accessors(self, class_name: str, properties: list) -> str:
        lines = ['        // Getters and Setters']
        for prop in properties:
            prop_name = prop['name']
            member_name = prop['member_name']
            prop_type = prop['type']
            clean_name = prop_name[2:] if prop_name.startswith('m_') else prop_name
            getter_name = f'Get{clean_name[0].upper()}{clean_name[1:]}'
            setter_name = f'Set{clean_name[0].upper()}{clean_name[1:]}'
            
            lines.append(f'        //! Get {camel_to_human(clean_name)}')
            lines.append(f'        const {prop_type}& {getter_name}() const {{ return {member_name}; }}')
            if not prop.get('read_only', False):
                lines.append(f'        //! Set {camel_to_human(clean_name)}')
                lines.append(f'        void {setter_name}(const {prop_type}& value) {{ {member_name} = value; }}')
            lines.append('')
        return '\n'.join(lines)
    
    def _render_property_members(self, properties: list) -> str:
        lines = []
        for prop in properties:
            member_name = prop['member_name']
            prop_type = prop['type']
            default = prop.get('default', '')
            if default:
                lines.append(f'        {prop_type} {member_name} = {default};')
            else:
                lines.append(f'        {prop_type} {member_name}{{}};')
        return '\n'.join(lines)
    
    def _render_struct_members(self, properties: list) -> str:
        lines = []
        for prop in properties:
            member_name = prop['member_name']
            prop_type = prop['type']
            default = prop.get('default', '')
            if default:
                lines.append(f'        {prop_type} {member_name} = {default};')
            else:
                lines.append(f'        {prop_type} {member_name}{{}};')
        return '\n'.join(lines)
    
    def _render_function_declarations(self, functions: list) -> str:
        lines = []
        for func in functions:
            func_name = func['name']
            return_type = func.get('return_type', 'void')
            params = ', '.join([
                f"{p['type']} {p['name']}" + (f" = {p['default']}" if p.get('default') else '')
                for p in func.get('parameters', [])
            ])
            tooltip = func.get('tooltip', func_name)
            lines.append(f'        //! {tooltip}')
            lines.append(f'        {return_type} {func_name}({params});')
            lines.append('')
        return '\n'.join(lines)
    
    def _render_serialize_fields(self, class_name: str, properties: list) -> str:
        lines = []
        for prop in properties:
            prop_name = prop['name']
            member_name = prop['member_name']
            lines.append(f'                ->Field("{prop_name}", &{class_name}::{member_name})')
        return '\n'.join(lines)
    
    def _render_edit_class_attributes(self, component: dict) -> str:
        lines = []
        if component.get('icon'):
            lines.append(f'                        ->Attribute(AZ::Edit::Attributes::Icon, "{component["icon"]}")')
        if not component.get('hide_in_editor', False):
            menu_cat = component.get('menu_category', 'Game')
            lines.append(f'                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("{menu_cat}"))')
        lines.append('                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)')
        return '\n'.join(lines)
    
    def _render_edit_data_elements(self, class_name: str, properties: list) -> str:
        lines = []
        current_category = ''
        for prop in properties:
            prop_name = prop['name']
            member_name = prop['member_name']
            display_name = prop.get('display_name', camel_to_human(prop_name.lstrip('m_')))
            tooltip = prop.get('tooltip', '')
            category = prop.get('category', '')
            
            # Add category group if changed
            if category and category != current_category:
                lines.append(f'                    ->ClassElement(AZ::Edit::ClassElements::Group, "{category}")')
                lines.append('                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)')
                current_category = category
            
            lines.append(f'                    ->DataElement(AZ::Edit::UIHandlers::Default, &{class_name}::{member_name}, "{display_name}", "{tooltip}")')
            
            # Add property attributes
            if prop.get('suffix'):
                lines.append(f'                        ->Attribute(AZ::Edit::Attributes::Suffix, "{prop["suffix"]}")')
            if prop.get('min') is not None:
                lines.append(f'                        ->Attribute(AZ::Edit::Attributes::Min, {prop["min"]})')
            if prop.get('max') is not None:
                lines.append(f'                        ->Attribute(AZ::Edit::Attributes::Max, {prop["max"]})')
            if prop.get('read_only'):
                lines.append('                        ->Attribute(AZ::Edit::Attributes::ReadOnly, true)')
        
        return '\n'.join(lines)
    
    def _render_behavior_properties(self, class_name: str, properties: list) -> str:
        lines = []
        for prop in properties:
            if not prop.get('expose_to_script', True) and not prop.get('blueprint_read_write', False):
                continue
            
            prop_name = prop['name']
            member_name = prop['member_name']
            prop_type = prop['type']
            clean_name = prop_name[2:] if prop_name.startswith('m_') else prop_name
            getter_name = f'Get{clean_name[0].upper()}{clean_name[1:]}'
            setter_name = f'Set{clean_name[0].upper()}{clean_name[1:]}'
            
            if prop.get('blueprint_read_only', False):
                lines.append(f'                ->Property("{clean_name}",')
                lines.append(f'                    []({class_name}* self) {{ return self->{getter_name}(); }},')
                lines.append('                    nullptr)')
            else:
                lines.append(f'                ->Property("{clean_name}",')
                lines.append(f'                    []({class_name}* self) {{ return self->{getter_name}(); }},')
                lines.append(f'                    []({class_name}* self, const {prop_type}& value) {{ self->{setter_name}(value); }})')
        
        return '\n'.join(lines)
    
    def _render_behavior_methods(self, class_name: str, functions: list) -> str:
        lines = []
        for func in functions:
            if not func.get('blueprint_callable', True):
                continue
            
            func_name = func['name']
            category = func.get('category', '')
            lines.append(f'                ->Method("{func_name}", &{class_name}::{func_name})')
            if category:
                lines.append(f'                    ->Attribute(AZ::Script::Attributes::Category, "{category}")')
        
        return '\n'.join(lines)
    
    def _render_service_functions(self, class_name: str, services: dict) -> str:
        provides = services.get('provides', [])
        incompatible = services.get('incompatible', [])
        requires = services.get('requires', [])
        dependent = services.get('dependent', [])
        
        return f'''
    void {class_name}::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {{
{self._render_service_list('provided', provides)}
    }}

    void {class_name}::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {{
{self._render_service_list('incompatible', incompatible)}
    }}

    void {class_name}::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {{
{self._render_service_list('required', requires)}
    }}

    void {class_name}::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {{
{self._render_service_list('dependent', dependent)}
    }}
'''
    
    def _render_service_list(self, var_name: str, services: list) -> str:
        if not services:
            return f'        (void){var_name}; // No services'
        lines = []
        for service in services:
            lines.append(f'        {var_name}.push_back(AZ_CRC_CE("{service}"));')
        return '\n'.join(lines)
    
    def _render_function_implementations(self, class_name: str, functions: list) -> str:
        lines = []
        for func in functions:
            func_name = func['name']
            return_type = func.get('return_type', 'void')
            params = ', '.join([f"{p['type']} {p['name']}" for p in func.get('parameters', [])])
            
            lines.append(f'    {return_type} {class_name}::{func_name}({params})')
            lines.append('    {')
            lines.append(f'        // TODO: Implement {func_name}')
            if return_type != 'void':
                lines.append('        return {};')
            lines.append('    }')
            lines.append('')
        
        return '\n'.join(lines)
    
    def _render_default_function_implementations(self, class_name: str, functions: list) -> str:
        """Render default empty implementations for functions (used when no _Impl.inl exists)."""
        lines = []
        for func in functions:
            func_name = func['name']
            return_type = func.get('return_type', 'void')
            params = ', '.join([f"{p['type']} /*{p['name']}*/" for p in func.get('parameters', [])])
            
            lines.append(f'    {return_type} {class_name}::{func_name}({params})')
            lines.append('    {')
            if return_type != 'void':
                lines.append('        return {};')
            lines.append('    }')
        
        return '\n'.join(lines)
    
    def _render_user_implementations(self, class_name: str, full_class_name: str, 
                                      user_functions: dict, declared_functions: list) -> str:
        """
        Render function implementations, using user code where available.
        
        Args:
            class_name: Short class name (e.g., "TestReflectionComponent")
            full_class_name: Full class name with namespace (e.g., "NewProject::TestReflectionComponent")
            user_functions: Dict of user-provided function implementations from .cpp
            declared_functions: List of O3DE_FUNCTION declared functions
        """
        lines = []
        lines.append('    // ========================================================================')
        lines.append('    // Component Lifecycle Functions')
        lines.append('    // ========================================================================')
        lines.append('')
        
        # List of standard component functions to look for
        lifecycle_functions = ['Init', 'Activate', 'Deactivate']
        
        for func_name in lifecycle_functions:
            # Try to find user implementation
            user_impl = None
            for key in [f"{class_name}::{func_name}", f"{full_class_name}::{func_name}"]:
                if key in user_functions:
                    user_impl = user_functions[key]
                    break
            
            if user_impl:
                # Use user's implementation
                lines.append(f"    void {class_name}::{func_name}()")
                lines.append('    {')
                # Indent the user's body
                for line in user_impl['body'].split('\n'):
                    lines.append(f"        {line}" if line.strip() else '')
                lines.append('    }')
            else:
                # Provide empty default
                lines.append(f"    void {class_name}::{func_name}()")
                lines.append('    {')
                lines.append(f'        // TODO: Implement {func_name}')
                lines.append('    }')
            lines.append('')
        
        # Now handle O3DE_FUNCTION declared functions
        if declared_functions:
            lines.append('    // ========================================================================')
            lines.append('    // Custom Functions')
            lines.append('    // ========================================================================')
            lines.append('')
            
            for func in declared_functions:
                func_name = func['name']
                return_type = func.get('return_type', 'void')
                params = ', '.join([f"{p['type']} {p['name']}" for p in func.get('parameters', [])])
                
                # Try to find user implementation
                user_impl = None
                for key in [f"{class_name}::{func_name}", f"{full_class_name}::{func_name}"]:
                    if key in user_functions:
                        user_impl = user_functions[key]
                        break
                
                const_suffix = ' const' if func.get('is_const', False) else ''
                
                if user_impl:
                    # Use user's implementation
                    lines.append(f"    {return_type} {class_name}::{func_name}({params}){const_suffix}")
                    lines.append('    {')
                    for line in user_impl['body'].split('\n'):
                        lines.append(f"        {line}" if line.strip() else '')
                    lines.append('    }')
                else:
                    # Provide stub default
                    lines.append(f"    {return_type} {class_name}::{func_name}({params}){const_suffix}")
                    lines.append('    {')
                    lines.append(f'        // TODO: Implement {func_name}')
                    if return_type != 'void':
                        lines.append('        return {};')
                    lines.append('    }')
                lines.append('')
        
        return '\n'.join(lines)
    
    def _render_struct_behavior_context(self, struct: dict) -> str:
        if not struct.get('blueprint_type', True):
            return ''
        
        struct_name = struct['name']
        category = struct.get('category', 'Data')
        properties = struct.get('properties', [])
        
        lines = [
            '        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))',
            '        {',
            f'            behaviorContext->Class<{struct_name}>("{struct_name}")',
            f'                ->Attribute(AZ::Script::Attributes::Category, "{category}")',
            '                ->Constructor()',
        ]
        
        for prop in properties:
            if not prop.get('expose_to_script', True):
                continue
            member_name = prop['member_name']
            clean_name = prop['name'][2:] if prop['name'].startswith('m_') else prop['name']
            lines.append(f'                ->Property("{clean_name}", BehaviorValueProperty(&{struct_name}::{member_name}))')
        
        lines.append('                ;')
        lines.append('        }')
        
        return '\n'.join(lines)
    
    def _render_struct_equality(self, properties: list) -> str:
        if not properties:
            return '        return true;'
        
        lines = ['        return']
        for i, prop in enumerate(properties):
            member_name = prop['member_name']
            if i == 0:
                lines.append(f'            {member_name} == rhs.{member_name}')
            else:
                lines.append(f'            && {member_name} == rhs.{member_name}')
        lines.append('            ;')
        
        return '\n'.join(lines)
    
    def _render_enum_values(self, enum: dict) -> str:
        lines = []
        is_flags = enum.get('is_flags', False)
        counter = 0
        
        for val in enum.get('values', []):
            val_name = val['name']
            val_num = val.get('value')
            
            if val_num is not None:
                lines.append(f'        {val_name} = {val_num},')
            elif is_flags:
                lines.append(f'        {val_name} = 1 << {counter},')
                counter += 1
            else:
                lines.append(f'        {val_name},')
        
        return '\n'.join(lines)
    
    def _render_enum_bitwise_operators(self, enum: dict) -> str:
        if not enum.get('is_flags', False):
            return ''
        return f'    AZ_DEFINE_ENUM_BITWISE_OPERATORS({enum["name"]});\n'
    
    def _render_enum_behavior_values(self, enum: dict) -> str:
        enum_name = enum['name']
        category = enum.get('category', 'Enums')
        values = enum.get('values', [])
        
        if not values:
            return '            // No enum values defined'
        
        # Build the Enum template arguments
        value_casts = ',\n'.join([
            f'                static_cast<int>({enum_name}::{v["name"]})'
            for v in values
        ])
        
        lines = [
            '            behaviorContext->Enum<',
            value_casts,
            f'            >("{enum_name}")',
        ]
        
        for val in values:
            display_name = val.get('display_name', camel_to_human(val['name']))
            lines.append(f'                ->Value("{display_name}", static_cast<int>({enum_name}::{val["name"]}))')
        
        lines.append(f'                ->Attribute(AZ::Script::Attributes::Category, "{category}")')
        lines.append('                ;')
        
        return '\n'.join(lines)
    
    def _write_file(self, path: str, content: str):
        """Write content to file, creating directories as needed."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(content)
        logger.info(f"Generated: {path}")


def main():
    parser = argparse.ArgumentParser(description='O3DEReflect Code Generator')
    parser.add_argument('--input', '-i', required=True, help='Input XML file')
    parser.add_argument('--output-dir', '-o', required=True, help='Output directory')
    parser.add_argument('--template-dir', '-t', help='Template directory (for Jinja2 templates)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    parser.add_argument('--dry-run', '-n', action='store_true', help='Parse only, do not generate')
    
    args = parser.parse_args()
    
    if args.verbose:
        logger.setLevel(logging.DEBUG)
    
    # Parse input file
    logger.info(f"Parsing: {args.input}")
    xml_parser = O3DEReflectParser(args.input)
    parsed_data = xml_parser.parse()
    
    if args.verbose:
        logger.debug(f"Parsed data: {json.dumps(parsed_data, indent=2, default=str)}")
    
    if args.dry_run:
        logger.info("Dry run - no files generated")
        return 0
    
    # Generate code
    template_dir = args.template_dir or os.path.dirname(os.path.abspath(__file__))
    generator = O3DEReflectGenerator(template_dir, args.output_dir)
    generated_files = generator.generate(parsed_data, args.input)
    
    logger.info(f"Generated {len(generated_files)} files")
    return 0


if __name__ == '__main__':
    sys.exit(main())
