import re

import yaml
import numpy as np
from numpy import pi,random


def evaluate(expr):
    if isinstance(expr, list):
        return [evaluate(e) for e in expr]
    elif isinstance(expr, str) and expr.startswith('$'):
        return eval(expr[1:])
    else:
        return expr


def set_node_field(node, attr, val):
    field = node.getField(attr)
    field_type = field.getTypeName()
    if field_type[1:] == 'FNode':
        instanciate(val, field)
    else:
        set_field_value = {
            'SFBool': field.setSFBool,
            'SFInt32': field.setSFInt32,
            'SFFloat': field.setSFFloat,
            'SFVec2f': lambda v: field.setSFVec2f(list(v)),
            'SFVec3f': lambda v: field.setSFVec3f(list(v)),
            'SFRotation': lambda v: field.setSFRotation(list(v)),
            'SFColor': lambda v: field.setSFColor(list(v)),
            'SFString': field.setSFString,
            'MFBool': field.insertMFBool,
            'MFInt32': field.insertMFInt32,
            'MFFloat': field.insertMFFloat,
            'MFVec2f': lambda i, v: field.insertMFVec2f(i, list(v)),
            'MFVec3f': lambda i, v: field.insertMFVec3f(i, list(v)),
            'MFRotation': lambda i, v: field.insertMFRotation(i, list(v)),
            'MFColor': lambda i, v: field.insertMFColor(i, list(v)),
            'MFString': field.insertMFString
        }[field_type]
        if field_type.startswith('M'): # multi-field
            for v in val:
                set_field_value(-1, evaluate(v))
        else: # single-field
            set_field_value(evaluate(val))


def get_node_field(node, attr):
    field = node.getField(attr)
    field_type = field.getTypeName()
    get_field_value = {
        'SFBool': field.getSFBool,
        'SFInt32': field.getSFInt32,
        'SFFloat': field.getSFFloat,
        'SFVec2f': field.getSFVec2f,
        'SFVec3f': field.getSFVec3f,
        'SFRotation': field.getSFRotation,
        'SFColor': field.getSFColor,
        'SFString': field.getSFString,
        'SFNode': field.getSFNode,
        'MFBool': field.getMFBool,
        'MFInt32': field.getMFInt32,
        'MFFloat': field.getMFFloat,
        'MFVec2f': field.getMFVec2f,
        'MFVec3f': field.getMFVec3f,
        'MFRotation': field.getMFRotation,
        'MFColor': field.getMFColor,
        'MFString': field.getMFString,
        'MFNode': field.getMFNode
    }[field_type]
    if field_type.startswith('M'): # multi-field
        return [get_field_value(i) for i in range(field.getCount())]
    else: # single-field
        return get_field_value()


def instanciate(nodes, parentField):
    if not isinstance(nodes, list):
        nodes = [nodes] # to handle single-field case
    
    # Iterate over nodes
    for node in nodes:
        name = node.get('_name', '')
        
        # Create as many instances of this node as required
        for i in range(evaluate(node.get('_number', 1))):
        
            # Insert node instance
            assert '_type' in node, f'{name} node has no type (please add "_type" attribute)'
            is_multi_field = parentField.getTypeName().startswith('M')
            if is_multi_field: # multi-field
                nodeDef = f'DEF {name}({i}) ' if name else ''
                parentField.importMFNodeFromString(-1, f'{nodeDef}{node["_type"]} {{}}')
            else: # single-field
                nodeDef = f'DEF {name} ' if name else ''
                parentField.importSFNodeFromString(f'{nodeDef}{node["_type"]} {{}}')

            # Save new node reference attached to its parent (for further use like export)
            new_node = parentField.getMFNode(-1) if is_multi_field else parentField.getSFNode()
            new_parent_id = new_node.getParentNode().getId()
            node['__ref'] = node.get('__ref', {})
            node['__ref'][new_parent_id] = node['__ref'].get(new_parent_id, []) + [new_node]

            # Set attribute values
            for attr, val in node.items():
                
                # Filter out non-Webots attributes
                if not attr.startswith('_'):
                    set_node_field(new_node, attr, val)


def extract(nodes, parentNode):
    is_single_node = not isinstance(nodes, list)
    if is_single_node:
        nodes = [nodes] # to handle single-field case
    
    # Iterate over nodes
    extracted = []
    for node in nodes:
        refs = node['__ref']
        for ref in refs.get(parentNode.getId(), []):
            node_extracted = {}

            # Get attribute values
            for attr, val in node.items():
                
                # Check if Webots attribute
                if not attr.startswith('_'):

                    # Check if field is of type node(s)
                    if ref.getField(attr).getTypeName()[1:] == 'FNode':
                        node_extracted[attr] = extract(val, ref)
                    else:
                        node_extracted[attr] = get_node_field(ref, attr)
                
                else:
                    get_special_field_value = {
                        '_name': lambda n, r: r.getDef(),
                        '_type': lambda n, r: n['_type']
                    }.get(attr, lambda n, r: None)
                    special_field_value = get_special_field_value(node, ref)
                    if special_field_value:
                        node_extracted[attr] = special_field_value
            
            extracted.append(node_extracted)
    
    return extracted[0] if len(extracted) == 1 and is_single_node else extracted


def generate_nodes(supervisor, spec, warmup=5, export=None):
    # Load spec (yaml file) if not a dict already
    if isinstance(spec, str):
        with open(spec) as f:
            spec = yaml.load(f, Loader=yaml.FullLoader)

    # (Re)create _GENERATED node (group containing all nodes to spawn)
    genNode = supervisor.getFromDef('_GENERATED')
    if genNode is not None:
        genNode.remove()
    rootNode = supervisor.getRoot()  # get root of the scene tree
    rootChildrenField = rootNode.getField('children')
    rootChildrenField.importMFNodeFromString(-1, f'DEF _GENERATED Group {{}}')
    genNode = rootChildrenField.getMFNode(-1)
    genChildrenField = genNode.getField('children')

    # Instanciate nodes
    nodes = spec.get('nodes', [])
    instanciate(nodes, genChildrenField)

    # Wait for warmup time (typically for objects to settle down)
    timestep = int(supervisor.getBasicTimeStep())
    t = 0
    while supervisor.step(timestep) != -1:
        t += timestep/1000
        if t >= warmup:
            break
    
    # Export instanciated nodes as yaml file
    if export:
        scene = {'nodes': extract(nodes, genNode)}
        with open(export, 'w') as f:
            yaml.dump(scene, f)
