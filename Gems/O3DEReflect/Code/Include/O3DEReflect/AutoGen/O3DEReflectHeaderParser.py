#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

"""
O3DEReflect Header Parser (V2)

This module parses C++ header files for O3DE_* macros (similar to Unreal's
UPROPERTY/UCLASS/UFUNCTION system) and generates reflection code automatically.

Supports:
    O3DE_CLASS(...)     - Mark a class for reflection
    O3DE_COMPONENT(...) - Component-specific metadata
    O3DE_PROPERTY(...)  - Mark members for serialization/editor
    O3DE_FUNCTION(...)  - Expose methods to scripting
    O3DE_STRUCT(...)    - Mark structs for reflection
    O3DE_ENUM(...)      - Mark enums for reflection
    O3DE_ENUM_VALUE(...) - Enum value metadata

Usage:
    python O3DEReflectHeaderParser.py --input MyComponent.h --output-dir ./Generated
"""

import os
import re
import sys
import json
import glob
import argparse
import hashlib
import logging
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Any, Tuple

# Try to find O3DE's Python venv site-packages for Jinja2
def find_o3de_site_packages():
    """Find O3DE's Python venv site-packages directory."""
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

logging.basicConfig(format='[%(levelname)s] %(name)s: %(message)s')
logger = logging.getLogger('O3DEReflectHeaderParser')
logger.setLevel(logging.INFO)


def create_hash_guid(string: str) -> str:
    """Generate a deterministic UUID from a string using MD5 hash."""
    hash_obj = hashlib.md5(string.encode('utf-8'))
    hash_str = hash_obj.hexdigest()
    return "{{{}-{}-{}-{}-{}}}".format(
        hash_str[0:8].upper(),
        hash_str[8:12].upper(),
        hash_str[12:16].upper(),
        hash_str[16:20].upper(),
        hash_str[20:].upper()
    )


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


@dataclass
class MacroAttribute:
    """Represents a parsed macro attribute (e.g., Category="Movement")"""
    name: str
    value: Any = None
    is_flag: bool = False  # True for bare flags like "EditAnywhere"


@dataclass
class PropertyInfo:
    """Parsed property information"""
    name: str
    cpp_type: str
    default_value: str = ""
    attributes: Dict[str, Any] = field(default_factory=dict)
    line_number: int = 0


@dataclass
class FunctionInfo:
    """Parsed function information"""
    name: str
    return_type: str = "void"
    parameters: List[Dict[str, str]] = field(default_factory=list)
    attributes: Dict[str, Any] = field(default_factory=dict)
    line_number: int = 0


@dataclass
class EnumValueInfo:
    """Parsed enum value information"""
    name: str
    value: Optional[int] = None
    attributes: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ClassInfo:
    """Parsed class/struct information"""
    name: str
    namespace: str = ""
    base_classes: List[str] = field(default_factory=list)
    is_component: bool = False
    is_struct: bool = False
    is_enum: bool = False
    uuid: str = ""
    attributes: Dict[str, Any] = field(default_factory=dict)
    properties: List[PropertyInfo] = field(default_factory=list)
    functions: List[FunctionInfo] = field(default_factory=list)
    enum_values: List[EnumValueInfo] = field(default_factory=list)
    services: Dict[str, List[str]] = field(default_factory=dict)
    includes: List[str] = field(default_factory=list)
    line_number: int = 0


class O3DEHeaderParser:
    """
    Parser for C++ headers containing O3DE_* reflection macros.
    
    This parser uses regex-based tokenization to extract macro annotations
    and their associated C++ declarations.
    """
    
    # Regex patterns for macro parsing
    MACRO_PATTERN = re.compile(
        r'O3DE_(CLASS|COMPONENT|PROPERTY|FUNCTION|STRUCT|ENUM|ENUM_VALUE|GENERATED_BODY)\s*\(([^)]*)\)',
        re.MULTILINE
    )
    
    # Pattern to match class/struct declaration after O3DE_CLASS/O3DE_STRUCT
    CLASS_DECL_PATTERN = re.compile(
        r'(?:class|struct)\s+(?:\[\[.*?\]\]\s*)?(\w+)\s*(?:final\s*)?(?::\s*(.+?))?\s*\{',
        re.MULTILINE | re.DOTALL
    )
    
    # Pattern to match enum declaration after O3DE_ENUM
    ENUM_DECL_PATTERN = re.compile(
        r'enum\s+(?:class\s+)?(\w+)\s*(?::\s*(\w+))?\s*\{([^}]*)\}',
        re.MULTILINE | re.DOTALL
    )
    
    # Pattern to match member variable declaration after O3DE_PROPERTY
    MEMBER_PATTERN = re.compile(
        r'^\s*((?:const\s+)?(?:[\w:]+(?:<[^>]+>)?(?:\s*[*&])?)+)\s+(\w+)\s*(?:=\s*([^;]+))?\s*;',
        re.MULTILINE
    )
    
    # Pattern to match function declaration after O3DE_FUNCTION
    FUNCTION_PATTERN = re.compile(
        r'^\s*((?:virtual\s+)?(?:static\s+)?(?:const\s+)?(?:[\w:]+(?:<[^>]+>)?(?:\s*[*&])?)+)\s+(\w+)\s*\(([^)]*)\)\s*(?:const)?\s*(?:override)?\s*(?:=\s*0)?\s*;',
        re.MULTILINE
    )
    
    # Pattern to parse macro attributes
    ATTR_PATTERN = re.compile(
        r'(\w+)\s*(?:=\s*(?:"([^"]*)"|([^,\s]+)))?',
        re.MULTILINE
    )
    
    # Pattern to match namespace
    NAMESPACE_PATTERN = re.compile(
        r'namespace\s+(\w+)\s*\{',
        re.MULTILINE
    )
    
    # Pattern to match includes
    INCLUDE_PATTERN = re.compile(
        r'#include\s*[<"]([^>"]+)[>"]',
        re.MULTILINE
    )
    
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.content = ""
        self.lines = []
        self.classes: List[ClassInfo] = []
        self.current_namespace = ""
        
    def parse(self) -> Dict:
        """Parse the header file and return structured data."""
        with open(self.file_path, 'r', encoding='utf-8') as f:
            self.content = f.read()
        self.lines = self.content.split('\n')
        
        # Extract includes
        includes = self.INCLUDE_PATTERN.findall(self.content)
        
        # Find all namespaces
        namespaces = list(self.NAMESPACE_PATTERN.finditer(self.content))
        
        # Find all O3DE_* macros
        macros = list(self.MACRO_PATTERN.finditer(self.content))
        
        # Process macros in order
        current_class: Optional[ClassInfo] = None
        pending_macro: Optional[Tuple[str, Dict, int]] = None
        
        for macro in macros:
            macro_type = macro.group(1)
            macro_args = macro.group(2)
            macro_pos = macro.start()
            macro_line = self.content[:macro_pos].count('\n') + 1
            
            # Determine current namespace
            current_ns = ""
            for ns_match in namespaces:
                if ns_match.start() < macro_pos:
                    current_ns = ns_match.group(1)
            
            attributes = self._parse_macro_attributes(macro_args)
            
            if macro_type == 'CLASS':
                # Look for class declaration after macro
                remaining = self.content[macro.end():]
                class_match = self.CLASS_DECL_PATTERN.search(remaining)
                if class_match:
                    class_name = class_match.group(1)
                    base_classes_str = class_match.group(2) or ""
                    base_classes = self._parse_base_classes(base_classes_str)
                    
                    full_name = f"{current_ns}::{class_name}" if current_ns else class_name
                    
                    current_class = ClassInfo(
                        name=class_name,
                        namespace=current_ns,
                        base_classes=base_classes,
                        is_component=any('Component' in bc for bc in base_classes),
                        uuid=attributes.get('Uuid', create_hash_guid(full_name)),
                        attributes=attributes,
                        includes=includes.copy(),
                        line_number=macro_line,
                    )
                    current_class.services = {
                        'provides': [],
                        'requires': [],
                        'incompatible': [],
                        'dependent': [],
                    }
                    self.classes.append(current_class)
                    
            elif macro_type == 'STRUCT':
                # Look for struct declaration after macro
                remaining = self.content[macro.end():]
                class_match = self.CLASS_DECL_PATTERN.search(remaining)
                if class_match:
                    struct_name = class_match.group(1)
                    full_name = f"{current_ns}::{struct_name}" if current_ns else struct_name
                    
                    current_class = ClassInfo(
                        name=struct_name,
                        namespace=current_ns,
                        is_struct=True,
                        uuid=attributes.get('Uuid', create_hash_guid(full_name)),
                        attributes=attributes,
                        includes=includes.copy(),
                        line_number=macro_line,
                    )
                    self.classes.append(current_class)
                    
            elif macro_type == 'ENUM':
                # Look for enum declaration after macro
                remaining = self.content[macro.end():]
                enum_match = self.ENUM_DECL_PATTERN.search(remaining)
                if enum_match:
                    enum_name = enum_match.group(1)
                    underlying_type = enum_match.group(2) or "int32_t"
                    enum_body = enum_match.group(3)
                    
                    full_name = f"{current_ns}::{enum_name}" if current_ns else enum_name
                    
                    enum_class = ClassInfo(
                        name=enum_name,
                        namespace=current_ns,
                        is_enum=True,
                        uuid=attributes.get('Uuid', create_hash_guid(full_name)),
                        attributes=attributes,
                        line_number=macro_line,
                    )
                    enum_class.attributes['UnderlyingType'] = underlying_type
                    
                    # Parse enum values
                    enum_class.enum_values = self._parse_enum_values(enum_body)
                    self.classes.append(enum_class)
                    
            elif macro_type == 'COMPONENT':
                # O3DE_COMPONENT adds component-specific info to current class
                if current_class and current_class.is_component:
                    current_class.is_component = True
                    # Parse services from attributes
                    if 'ProvidesServices' in attributes:
                        current_class.services['provides'] = self._parse_service_list(attributes['ProvidesServices'])
                    if 'RequiresServices' in attributes:
                        current_class.services['requires'] = self._parse_service_list(attributes['RequiresServices'])
                    if 'IncompatibleServices' in attributes:
                        current_class.services['incompatible'] = self._parse_service_list(attributes['IncompatibleServices'])
                    if 'DependentServices' in attributes:
                        current_class.services['dependent'] = self._parse_service_list(attributes['DependentServices'])
                    # Merge other attributes
                    current_class.attributes.update(attributes)
                    
            elif macro_type == 'PROPERTY':
                # Store pending property macro, look for member declaration
                pending_macro = ('PROPERTY', attributes, macro.end())
                
            elif macro_type == 'FUNCTION':
                # Store pending function macro, look for function declaration
                pending_macro = ('FUNCTION', attributes, macro.end())
                
            elif macro_type == 'GENERATED_BODY':
                # Marker for where generated code should be included
                if current_class:
                    current_class.attributes['_generated_body_line'] = macro_line
        
        # Now do a second pass to find member and function declarations
        self._parse_members_and_functions()
        
        return self._to_dict()
    
    def _parse_macro_attributes(self, args_str: str) -> Dict[str, Any]:
        """Parse macro attributes like Category="Movement", EditAnywhere, Min=0.0"""
        attributes = {}
        if not args_str.strip():
            return attributes
        
        # Split by comma, but be careful with nested content
        parts = self._split_macro_args(args_str)
        
        for part in parts:
            part = part.strip()
            if not part:
                continue
            
            # Check for key=value pattern
            if '=' in part:
                key, value = part.split('=', 1)
                key = key.strip()
                value = value.strip().strip('"')
                
                # Try to convert to appropriate type
                if value.lower() in ('true', 'false'):
                    value = value.lower() == 'true'
                else:
                    try:
                        if '.' in value:
                            value = float(value.rstrip('f'))
                        else:
                            value = int(value)
                    except ValueError:
                        pass  # Keep as string
                        
                attributes[key] = value
            else:
                # Bare flag like "EditAnywhere"
                attributes[part] = True
        
        return attributes
    
    def _split_macro_args(self, args_str: str) -> List[str]:
        """Split macro arguments by comma, respecting nested parentheses and strings."""
        parts = []
        current = ""
        depth = 0
        in_string = False
        
        for char in args_str:
            if char == '"' and (not current or current[-1] != '\\'):
                in_string = not in_string
            elif not in_string:
                if char in '([{':
                    depth += 1
                elif char in ')]}':
                    depth -= 1
                elif char == ',' and depth == 0:
                    parts.append(current)
                    current = ""
                    continue
            current += char
        
        if current:
            parts.append(current)
        
        return parts
    
    def _parse_base_classes(self, base_str: str) -> List[str]:
        """Parse base class list from inheritance declaration."""
        if not base_str:
            return []
        
        bases = []
        for part in base_str.split(','):
            part = part.strip()
            # Remove access specifiers
            for spec in ['public', 'protected', 'private', 'virtual']:
                part = part.replace(spec, '').strip()
            if part:
                bases.append(part)
        
        return bases
    
    def _parse_service_list(self, services_str: str) -> List[str]:
        """Parse comma-separated service list."""
        if isinstance(services_str, list):
            return services_str
        return [s.strip().strip('"') for s in services_str.split(',') if s.strip()]
    
    def _parse_enum_values(self, enum_body: str) -> List[EnumValueInfo]:
        """Parse enum values from enum body."""
        values = []
        
        # Remove comments
        enum_body = re.sub(r'//.*$', '', enum_body, flags=re.MULTILINE)
        enum_body = re.sub(r'/\*.*?\*/', '', enum_body, flags=re.DOTALL)
        
        # Split by comma
        parts = [p.strip() for p in enum_body.split(',') if p.strip()]
        
        for part in parts:
            # Check for O3DE_ENUM_VALUE macro
            ev_match = re.search(r'O3DE_ENUM_VALUE\s*\(([^)]*)\)\s*(\w+)', part)
            if ev_match:
                attrs = self._parse_macro_attributes(ev_match.group(1))
                name = ev_match.group(2)
            else:
                # Parse regular enum value
                if '=' in part:
                    name, val = part.split('=', 1)
                    name = name.strip()
                    try:
                        attrs = {'Value': int(val.strip())}
                    except ValueError:
                        attrs = {'Value': val.strip()}
                else:
                    name = part.strip()
                    attrs = {}
            
            if name:
                values.append(EnumValueInfo(
                    name=name,
                    value=attrs.get('Value'),
                    attributes=attrs
                ))
        
        return values
    
    def _parse_members_and_functions(self):
        """Second pass: match O3DE_PROPERTY/FUNCTION macros to declarations."""
        for cls in self.classes:
            if cls.is_enum:
                continue
            
            # Find the class body
            class_start = self._find_class_body_start(cls.name, cls.line_number)
            if class_start < 0:
                continue
            
            class_body = self.content[class_start:]
            # Find matching closing brace
            brace_depth = 1
            class_end = 0
            for i, char in enumerate(class_body[1:], 1):
                if char == '{':
                    brace_depth += 1
                elif char == '}':
                    brace_depth -= 1
                    if brace_depth == 0:
                        class_end = i
                        break
            
            class_body = class_body[:class_end]
            
            # Find all O3DE_PROPERTY macros in class body
            # Format: O3DE_PROPERTY(member_name, Type = "...", ...)
            # Match multiline macros with parenthesis counting
            property_pattern = re.compile(r'O3DE_PROPERTY\s*\(', re.MULTILINE)
            for match in property_pattern.finditer(class_body):
                start = match.end()
                depth = 1
                end = start
                while end < len(class_body) and depth > 0:
                    if class_body[end] == '(':
                        depth += 1
                    elif class_body[end] == ')':
                        depth -= 1
                    end += 1
                
                if depth == 0:
                    macro_content = class_body[start:end-1]
                    # Parse: member_name, attr1 = val1, attr2 = val2
                    args = self._split_macro_args(macro_content)
                    if args:
                        member_name = args[0].strip()
                        attr_str = ', '.join(args[1:]) if len(args) > 1 else ''
                        attrs = self._parse_macro_attributes(attr_str)
                        
                        # Get type and default from attributes
                        prop_type = attrs.get('type', attrs.get('Type', 'AZ::s32'))
                        default_val = attrs.get('default', attrs.get('Default', ''))
                        
                        prop = PropertyInfo(
                            name=member_name,
                            cpp_type=prop_type,
                            default_value=default_val,
                            attributes=attrs,
                        )
                        cls.properties.append(prop)
            
            # Find all O3DE_FUNCTION macros in class body
            for match in re.finditer(r'O3DE_FUNCTION\s*\(([^)]*)\)\s*\n\s*(.+?);', class_body, re.MULTILINE):
                attrs = self._parse_macro_attributes(match.group(1))
                decl = match.group(2).strip()
                
                # Parse function declaration
                func_match = re.match(r'([\w:<>,\s*&]+?)\s+(\w+)\s*\(([^)]*)\)', decl)
                if func_match:
                    params = self._parse_function_params(func_match.group(3))
                    func = FunctionInfo(
                        name=func_match.group(2),
                        return_type=func_match.group(1).strip(),
                        parameters=params,
                        attributes=attrs,
                    )
                    cls.functions.append(func)
    
    def _find_class_body_start(self, class_name: str, start_line: int) -> int:
        """Find the position of the opening brace of a class."""
        # Search from the given line
        start_pos = sum(len(line) + 1 for line in self.lines[:start_line-1])
        remaining = self.content[start_pos:]
        
        # Find class declaration
        pattern = re.compile(rf'(?:class|struct)\s+{re.escape(class_name)}\s*(?:final\s*)?(?::[^{{]+)?\s*\{{')
        match = pattern.search(remaining)
        if match:
            return start_pos + match.end() - 1
        return -1
    
    def _parse_function_params(self, params_str: str) -> List[Dict[str, str]]:
        """Parse function parameters."""
        params = []
        if not params_str.strip():
            return params
        
        for part in self._split_macro_args(params_str):
            part = part.strip()
            if not part:
                continue
            
            # Parse "Type name = default" or "Type name"
            match = re.match(r'([\w:<>,\s*&]+?)\s+(\w+)\s*(?:=\s*(.+))?$', part)
            if match:
                params.append({
                    'type': match.group(1).strip(),
                    'name': match.group(2),
                    'default': match.group(3).strip() if match.group(3) else "",
                })
        
        return params
    
    def _to_dict(self) -> Dict:
        """Convert parsed data to dictionary format compatible with generator."""
        result = {
            'components': [],
            'structs': [],
            'enums': [],
            'source_file': os.path.basename(self.file_path),
        }
        
        for cls in self.classes:
            if cls.is_enum:
                enum_data = self._enum_to_dict(cls)
                result['enums'].append(enum_data)
            elif cls.is_struct:
                struct_data = self._struct_to_dict(cls)
                result['structs'].append(struct_data)
            else:
                comp_data = self._component_to_dict(cls)
                result['components'].append(comp_data)
        
        return result
    
    def _component_to_dict(self, cls: ClassInfo) -> Dict:
        """Convert ClassInfo to component dictionary."""
        full_name = f"{cls.namespace}::{cls.name}" if cls.namespace else cls.name
        
        return {
            'name': cls.name,
            'namespace': cls.namespace,
            'full_name': full_name,
            'uuid': cls.uuid,
            'category': cls.attributes.get('Category', 'General'),
            'description': cls.attributes.get('Description', f'{cls.name} component'),
            'display_name': cls.attributes.get('DisplayName', camel_to_human(cls.name)),
            'icon': cls.attributes.get('Icon', ''),
            'hide_in_editor': cls.attributes.get('HideInEditor', False),
            'abstract': cls.attributes.get('Abstract', False),
            'menu_category': cls.attributes.get('AppearsInAddComponentMenu', 'Game'),
            'includes': cls.includes,
            'base_classes': [{'name': bc.split('::')[-1], 'namespace': '::'.join(bc.split('::')[:-1]) if '::' in bc else '', 'include': ''} for bc in cls.base_classes],
            'services': cls.services,
            'properties': [self._property_to_dict(p) for p in cls.properties],
            'functions': [self._function_to_dict(f) for f in cls.functions],
        }
    
    def _struct_to_dict(self, cls: ClassInfo) -> Dict:
        """Convert ClassInfo to struct dictionary."""
        full_name = f"{cls.namespace}::{cls.name}" if cls.namespace else cls.name
        
        return {
            'name': cls.name,
            'namespace': cls.namespace,
            'full_name': full_name,
            'uuid': cls.uuid,
            'category': cls.attributes.get('Category', 'Data'),
            'description': cls.attributes.get('Description', f'{cls.name} data structure'),
            'display_name': cls.attributes.get('DisplayName', camel_to_human(cls.name)),
            'blueprint_type': cls.attributes.get('BlueprintType', True),
            'atomic': cls.attributes.get('Atomic', False),
            'includes': cls.includes,
            'properties': [self._property_to_dict(p) for p in cls.properties],
        }
    
    def _enum_to_dict(self, cls: ClassInfo) -> Dict:
        """Convert ClassInfo to enum dictionary."""
        full_name = f"{cls.namespace}::{cls.name}" if cls.namespace else cls.name
        
        return {
            'name': cls.name,
            'namespace': cls.namespace,
            'full_name': full_name,
            'uuid': cls.uuid,
            'category': cls.attributes.get('Category', 'Enums'),
            'description': cls.attributes.get('Description', f'{cls.name} enumeration'),
            'display_name': cls.attributes.get('DisplayName', camel_to_human(cls.name)),
            'blueprint_type': cls.attributes.get('BlueprintType', True),
            'is_flags': cls.attributes.get('Flags', False),
            'underlying_type': cls.attributes.get('UnderlyingType', 'int32_t'),
            'values': [self._enum_value_to_dict(v) for v in cls.enum_values],
        }
    
    def _property_to_dict(self, prop: PropertyInfo) -> Dict:
        """Convert PropertyInfo to dictionary."""
        prop_name = prop.name
        return {
            'name': prop_name,
            'member_name': prop_name if prop_name.startswith('m_') else f'm_{prop_name}',
            'type': prop.cpp_type,
            'default': prop.default_value,
            'display_name': prop.attributes.get('DisplayName', camel_to_human(prop_name.lstrip('m_'))),
            'category': prop.attributes.get('Category', ''),
            'tooltip': prop.attributes.get('Tooltip', ''),
            'suffix': prop.attributes.get('Suffix', ''),
            'edit_anywhere': prop.attributes.get('EditAnywhere', False),
            'edit_defaults_only': prop.attributes.get('EditDefaultsOnly', False),
            'edit_instance_only': prop.attributes.get('EditInstanceOnly', False),
            'visible_anywhere': prop.attributes.get('VisibleAnywhere', False),
            'read_only': prop.attributes.get('ReadOnly', False),
            'blueprint_read_write': prop.attributes.get('BlueprintReadWrite', False),
            'blueprint_read_only': prop.attributes.get('BlueprintReadOnly', False),
            'expose_to_script': prop.attributes.get('ExposeToScript', True),
            'min': prop.attributes.get('Min'),
            'max': prop.attributes.get('Max'),
            'ui_min': prop.attributes.get('UIMin'),
            'ui_max': prop.attributes.get('UIMax'),
            'step': prop.attributes.get('Step'),
            'change_notify': prop.attributes.get('ChangeNotify', ''),
            'visibility': prop.attributes.get('Visibility', ''),
            'order': prop.attributes.get('Order', 0),
        }
    
    def _function_to_dict(self, func: FunctionInfo) -> Dict:
        """Convert FunctionInfo to dictionary."""
        return {
            'name': func.name,
            'display_name': func.attributes.get('DisplayName', camel_to_human(func.name)),
            'category': func.attributes.get('Category', ''),
            'tooltip': func.attributes.get('Tooltip', ''),
            'return_type': func.return_type,
            'parameters': func.parameters,
            'blueprint_callable': func.attributes.get('BlueprintCallable', True),
            'blueprint_pure': func.attributes.get('BlueprintPure', False),
            'call_in_editor': func.attributes.get('CallInEditor', False),
            'server': func.attributes.get('Server', False),
            'client': func.attributes.get('Client', False),
            'net_multicast': func.attributes.get('NetMulticast', False),
        }
    
    def _enum_value_to_dict(self, val: EnumValueInfo) -> Dict:
        """Convert EnumValueInfo to dictionary."""
        return {
            'name': val.name,
            'value': val.value,
            'display_name': val.attributes.get('DisplayName', camel_to_human(val.name)),
            'tooltip': val.attributes.get('Tooltip', ''),
            'hidden': val.attributes.get('Hidden', False),
        }


class CppSourceParser:
    """
    Parser for C++ source files to extract user function implementations.
    Used to merge user code with generated reflection boilerplate.
    """
    
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.content = ""
        self.functions = {}  # Dict of function_signature -> function_body
        self.includes = []
        self.extra_code = []  # Any code not recognized as a function implementation
        
    def parse(self) -> Dict:
        """Parse the source file and extract function implementations."""
        if not os.path.exists(self.file_path):
            logger.debug(f"Source file not found: {self.file_path}")
            return {'functions': {}, 'includes': [], 'extra_code': []}
        
        with open(self.file_path, 'r', encoding='utf-8', errors='ignore') as f:
            self.content = f.read()
        
        self._extract_includes()
        self._extract_functions()
        
        return {
            'functions': self.functions,
            'includes': self.includes,
            'extra_code': self.extra_code,
            'raw_content': self.content
        }
    
    def _extract_includes(self):
        """Extract #include statements."""
        include_pattern = re.compile(r'^\s*#include\s*[<"]([^>"]+)[>"]', re.MULTILINE)
        self.includes = include_pattern.findall(self.content)
    
    def _extract_functions(self):
        """Extract function implementations with their bodies."""
        # Pattern to match function definitions: ReturnType ClassName::FunctionName(params) { body }
        # This handles multi-line function bodies by tracking brace depth
        
        # First, remove comments to avoid matching code inside comments
        content_no_comments = self._remove_comments(self.content)
        
        # Pattern for function signature (simplified - handles common cases)
        func_pattern = re.compile(
            r'(?:^|\n)\s*'  # Start of line
            r'((?:[\w:*&<>,\s]+)?)'  # Return type (optional, can be complex like const Type&)
            r'\s+([\w:]+)::(\w+)'  # ClassName::FunctionName
            r'\s*\(([^)]*)\)'  # Parameters
            r'\s*(const)?'  # Optional const
            r'\s*(override)?'  # Optional override
            r'\s*\{',  # Opening brace
            re.MULTILINE
        )
        
        for match in func_pattern.finditer(content_no_comments):
            return_type = match.group(1).strip() if match.group(1) else 'void'
            class_name = match.group(2)
            func_name = match.group(3)
            params = match.group(4).strip()
            is_const = bool(match.group(5))
            
            # Find the matching closing brace
            start_pos = match.end() - 1  # Position of opening brace
            body_start = match.end()
            body_end = self._find_matching_brace(content_no_comments, start_pos)
            
            if body_end > body_start:
                body = content_no_comments[body_start:body_end].strip()
                
                # Create a key that identifies this function
                key = f"{class_name}::{func_name}"
                
                self.functions[key] = {
                    'return_type': return_type,
                    'class_name': class_name,
                    'func_name': func_name,
                    'params': params,
                    'is_const': is_const,
                    'body': body,
                    'full_signature': f"{return_type} {class_name}::{func_name}({params})" + (" const" if is_const else "")
                }
    
    def _remove_comments(self, code: str) -> str:
        """Remove C and C++ style comments from code."""
        # Remove single-line comments
        code = re.sub(r'//.*?$', '', code, flags=re.MULTILINE)
        # Remove multi-line comments
        code = re.sub(r'/\*.*?\*/', '', code, flags=re.DOTALL)
        return code
    
    def _find_matching_brace(self, code: str, start_pos: int) -> int:
        """Find the position of the closing brace that matches the opening brace at start_pos."""
        if start_pos >= len(code) or code[start_pos] != '{':
            return -1
        
        depth = 1
        pos = start_pos + 1
        in_string = False
        string_char = None
        
        while pos < len(code) and depth > 0:
            char = code[pos]
            prev_char = code[pos - 1] if pos > 0 else ''
            
            # Handle string literals
            if char in '"\'':
                if not in_string:
                    in_string = True
                    string_char = char
                elif char == string_char and prev_char != '\\':
                    in_string = False
            elif not in_string:
                if char == '{':
                    depth += 1
                elif char == '}':
                    depth -= 1
            
            pos += 1
        
        return pos - 1 if depth == 0 else -1


def parse_source(file_path: str) -> Dict:
    """Parse a C++ source file and return extracted function implementations."""
    parser = CppSourceParser(file_path)
    return parser.parse()


def parse_header(file_path: str) -> Dict:
    """Parse a header file and return structured data."""
    parser = O3DEHeaderParser(file_path)
    return parser.parse()


def main():
    parser = argparse.ArgumentParser(description='O3DEReflect Header Parser')
    parser.add_argument('--input', '-i', required=True, help='Input header file')
    parser.add_argument('--output-dir', '-o', help='Output directory for generated files')
    parser.add_argument('--output-json', '-j', help='Output JSON file with parsed data')
    parser.add_argument('--template-dir', '-t', help='Template directory (defaults to script directory)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    if args.verbose:
        logger.setLevel(logging.DEBUG)
    
    # Parse header
    logger.info(f"Parsing: {args.input}")
    parsed_data = parse_header(args.input)
    
    # Also parse the corresponding .cpp file if it exists
    cpp_file = args.input.replace('.h', '.cpp')
    user_code = parse_source(cpp_file)
    if user_code['functions']:
        logger.info(f"Found user implementations in: {cpp_file}")
        parsed_data['user_code'] = user_code
    
    if args.verbose:
        logger.debug(f"Parsed data: {json.dumps(parsed_data, indent=2, default=str)}")
    
    # Output JSON if requested
    if args.output_json:
        with open(args.output_json, 'w', encoding='utf-8') as f:
            json.dump(parsed_data, f, indent=2, default=str)
        logger.info(f"Wrote JSON: {args.output_json}")
    
    # Generate code if output directory specified
    if args.output_dir:
        # Import the generator from O3DEReflectGen
        script_dir = os.path.dirname(os.path.abspath(__file__))
        template_dir = args.template_dir if args.template_dir else script_dir
        sys.path.insert(0, script_dir)
        from O3DEReflectGen import O3DEReflectGenerator
        
        generator = O3DEReflectGenerator(template_dir, args.output_dir)
        generated_files = generator.generate(parsed_data, args.input)
        logger.info(f"Generated {len(generated_files)} files")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
