#!/usr/bin/python
import re
import sys

from definition import *
from string import Template

# Converts text in 'CamelCase' into 'CAMEL_CASE'
# Snippet taken from: http://stackoverflow.com/questions/1175208/elegant-python-function-to-convert-camelcase-to-camel-case
def convertToUnderscore(s):
    return re.sub('(?!^)([0-9A-Z]+)', r'_\1', s).upper().replace('__', '_')

def convertToCamelCase(s):
    result = ''
    begin = True
    for c in s:
        if c == '_':
            begin = True
        elif begin:
            result += c.upper()
            begin = False
        else:
            result += c
    return result

def nameListToString(nameList):
    nameString = ''
    for name in nameList:
        if len(nameString) > 0:
            nameString += ' '
        nameString += name
    return nameString

class MakePharoBindingsVisitor:
    def __init__(self, out):
        self.out = out
        self.variables = {}
        self.constants = {}
        self.typeBindings = {}
        self.interfaceTypeMap = {}

    def processText(self, text, **extraVariables):
        t = Template(text)
        return t.substitute(**dict(self.variables.items() + extraVariables.items()))

    def write(self, text):
        self.out.write(text)

    def writeLine(self, line):
        self.write(line)
        self.newline()

    def newline(self):
        self.write('\n')

    def printString(self, text, **extraVariables):
        self.write(self.processText(text, **extraVariables))

    def printLine(self, text, **extraVariables):
        self.write(self.processText(text, **extraVariables))
        self.newline()

    def visitApiDefinition(self, api):
        self.setup(api)
        self.processVersions(api.versions)
        self.emitBindings(api)

    def setup(self, api):
        self.api = api
        self.variables ={
            'ConstantPrefix' : api.constantPrefix,
            'FunctionPrefix' : api.functionPrefix,
            'TypePrefix' : api.typePrefix,
        }

    def visitEnum(self, enum):
        for constant in enum.constants:
            cname = self.processText("$ConstantPrefix$ConstantName", ConstantName=convertToUnderscore(constant.name))
            self.constants[cname] = constant.value
        cenumName = self.processText("$TypePrefix$EnumName", EnumName=enum.name)
        self.typeBindings[cenumName] = '#int'

    def visitTypedef(self, typedef):
        mappingType = typedef.ctype
        if mappingType.startswith('const '):
            mappingType = mappingType[len('const '):]

        if mappingType.startswith('unsigned '):
            mappingType = 'u' + mappingType[len('unsigned '):]

        if mappingType.startswith('signed '):
            mappingType = mappingType[len('signed '):]

        typedefName = self.processText("$TypePrefix$Name", Name=typedef.name)
        self.typeBindings[typedefName] = "#'" + mappingType + "'"

    def visitInterface(self, interface):
        cname = typedefName = self.processText("$TypePrefix$Name", Name=interface.name)
        self.interfaceTypeMap[interface.name + '*'] = 'APHY' + convertToCamelCase(interface.name)
        self.typeBindings[cname] = "#'void'"

    def processFragment(self, fragment):
        # Visit the constants.
        for constant in fragment.constants:
            constant.accept(self)

        # Visit the types.
        for type in fragment.types:
            type.accept(self)

        # Visit the interfaces.
        for interface in fragment.interfaces:
            interface.accept(self)

    def processVersion(self, version):
        self.processFragment(version)

    def processVersions(self, versions):
        for version in versions.values():
            self.processVersion(version)
    def emitDoIt(self, string):
        self.printLine('!' + string + '!')

    def emitSubclass(self, baseClass, className, instanceVariableNames, classVariableNames, poolDictionaries):
        self.printLine("$BaseClass subclass: #$ClassName", BaseClass = baseClass, ClassName = className)
        self.printLine("\tinstanceVariableNames: '$InstanceVariableNames'", InstanceVariableNames = instanceVariableNames)
        self.printLine("\tclassVariableNames: '$ClassVariableNames'", ClassVariableNames = classVariableNames)
        self.printLine("\tpoolDictionaries: '$PoolDictionaries'", PoolDictionaries = poolDictionaries)
        self.printLine("\tcategory: 'AbstractPhysics-Generated'")
        self.printLine("!")
        self.newline()

    def emitConstants(self):
        self.emitSubclass('SharedPool', 'APHYConstants', '', nameListToString(self.constants.keys()), '')
        self.beginMethod('APHYConstants class', 'initialize')
        self.printLine('initialize')
        self.printLine('"')
        self.printLine('\tself initialize')
        self.printLine('"')
        self.printLine('\tsuper initialize.')
        self.newline()
        self.printString(
"""
    self data pairsDo: [:k :v |
        self writeClassVariableNamed: k value: v
    ]
""")
        self.endMethod()

        self.beginMethod('APHYConstants class', 'initialization')
        self.printLine('data')
        self.printLine('\t^ #(')
        for constantName in self.constants.keys():
            constantValue = self.constants[constantName]
            if constantValue.startswith('0x'):
                constantValue = '16r' + constantValue[2:]
            self.printLine("\t\t$ConstantName $ConstantValue" , ConstantName = constantName, ConstantValue = constantValue)
        self.printLine('\t)')
        self.endMethod()

    def emitTypeBindings(self):
        self.emitSubclass('SharedPool', 'APHYTypes', '', nameListToString(self.typeBindings.keys()), '')
        self.beginMethod('APHYTypes class', 'initialize')
        self.printLine('initialize')
        self.printLine('"')
        self.printLine('\tself initialize')
        self.printLine('"')
        self.printLine('\tsuper initialize.')
        self.newline()

        for ctypeName in self.typeBindings.keys():
            pharoName = self.typeBindings[ctypeName]
            self.printLine('\t$CTypeName := $PharoName.', CTypeName = ctypeName, PharoName = pharoName)
        self.endMethod()

    def emitCBindings(self, api):
        self.emitSubclass('APHYCBindingsBase', 'APHYCBindings', '', '', 'APHYConstants APHYTypes')

        for version in api.versions.values():
            # Emit the methods of the interfaces.
            for interface in version.interfaces:
                self.emitInterfaceCBindings(interface)

            # Emit the global c functions
            self.emitCGlobals(version.globals)

    def emitInterfaceCBindings(self, interface):
        for method in interface.methods:
            self.emitCMethodBinding(method, interface.name)

    def emitCGlobals(self, globals):
        for method in globals:
            self.emitCMethodBinding(method, 'global c functions')

    def emitCMethodBinding(self, method, category):
        self.beginMethod('APHYCBindings', category)
        self.printString("$MethodName", MethodName = method.name)

        allArguments = method.arguments
        if method.clazz is not None:
            allArguments = [SelfArgument(method.clazz)] + allArguments

        first = True
        for arg in allArguments:
            if first:
                self.printString('_')
                first = False
            else:
                self.printString(' ')

            name = arg.name
            if name == 'self': name = 'selfObject'
            self.printString("$ArgName: $ArgName", ArgName = name)

        self.newline()
        self.printString("\t^ self ffiCall: #($TypePrefix$ReturnType $FunctionPrefix$FunctionName (",
            ReturnType = method.returnType,
            FunctionName = method.cname)

        first = True
        for arg in allArguments:
            if first:
                first = False
            else:
                self.printString(' , ')

            name = arg.name
            if name == 'self': name = 'selfObject'
            argTypeString = arg.type
            if (arg.arrayReturn or arg.pointerList) and argTypeString.endswith('**'):
                argTypeString = argTypeString[:-1]
            self.printString("$TypePrefix$ArgType $ArgName", ArgType = argTypeString, ArgName = name)

        self.printLine(") )")
        self.endMethod()

    def emitInterfaceClasses(self, api):
        for version in api.versions.values():
            for interface in version.interfaces:
                pharoName = "APHY" + convertToCamelCase(interface.name)
                self.emitSubclass('APHYInterface', pharoName, '', '', '')

    def emitStructure(self, struct):
        cname = self.processText("$TypePrefix$StructName" , StructName = struct.name)
        pharoName = 'APHY' + convertToCamelCase(struct.name)
        self.typeBindings[cname] = pharoName
        self.emitSubclass('FFIExternalStructure', pharoName, '', '', 'APHYConstants APHYTypes')

        self.beginMethod(pharoName + ' class', 'definition')
        self.printLine(
"""fieldsDesc
	"
	self rebuildFieldAccessors
	"
	^ #(""")
        for field in struct.fields:
            self.printLine("\t\t $TypePrefix$FieldType $FieldName;", FieldType = field.type, FieldName = field.name)

        self.printLine("\t\t)")
        self.endMethod()

    def emitStructures(self, api):
        for version in api.versions.values():
            for struct in version.structs:
                self.emitStructure(struct)

    def emitInitializations(self, api):
        self.emitDoIt("""
        APHYTypes initialize.
        APHYConstants initialize.
        """)

    def emitStructuresInitializations(self, api):
        doItString = ''
        for version in api.versions.values():
            for struct in version.structs:
                pharoName = 'APHY' + convertToCamelCase(struct.name)
                doItString += pharoName + " rebuildFieldAccessors.\n"
        self.emitDoIt(doItString)

    def emitBaseClasses(self, api):
        self.emitSubclass('SharedPool', 'APHYTypes', '', '', '')
        self.emitConstants()
        self.emitInterfaceClasses(api)
        self.emitStructures(api)
        self.emitTypeBindings()
        self.emitCBindings(api)
        self.emitPharoBindings(api)
        self.emitInitializations(api)
        self.emitStructuresInitializations(api)

    def emitPharoBindings(self, api):
        for version in api.versions.values():
            for interface in version.interfaces:
                self.emitInterfaceBindings(interface)
            self.emitGlobals(version.globals)

    def emitInterfaceBindings(self, interface):
        for method in interface.methods:
            self.emitMethodWrapper(method)

    def emitGlobals(self, globals):
        for method in globals:
            self.emitMethodWrapper(method)

    def emitMethodWrapper(self, method):
        ownerClass = 'APHY'
        clazz = method.clazz
        allArguments = method.arguments
        category = '*AbstractPhysics-Generated'
        if clazz is not None:
            ownerClass = 'APHY' + convertToCamelCase(clazz.name)
            allArguments = [SelfArgument(method.clazz)] + allArguments
            category = 'wrappers'

        self.beginMethod(ownerClass, category)

        methodName = method.name
        if methodName == 'release':
            methodName = 'primitiveRelease'

        # Build the method selector.
        self.printString("$MethodName", MethodName = methodName)

        first = True
        for arg in method.arguments:
            name = arg.name
            if first:
                first = False
                self.printString(": $ArgName", ArgName = name)
            else:
                self.printString(" $ArgName: $ArgName", ArgName = name)
        self.newline()

        # Temporal variable for the return value
        self.printLine("\t| resultValue_ |")

        # Call the c bindings.
        self.printString("\tresultValue_ := APHYCBindings uniqueInstance $MethodName", MethodName = method.name)
        first = True
        for arg in allArguments:
            name = arg.name
            if name == 'self': name = 'selfObject'
            value = name

            if arg.type in self.interfaceTypeMap:
                value = self.processText("(self validHandleOf: $ArgName)", ArgName = name)

            if first:
                if first and clazz is not None:
                    self.printString('_$ArgName: (self validHandle)', ArgName = name)
                else:
                    self.printString('_$ArgName: $ArgValue', ArgName = name, ArgValue = value)
                first = False
            else:
                self.printString(' $ArgName: $ArgValue', ArgName = name, ArgValue = value)
        self.printLine('.')

        if method.returnType in self.interfaceTypeMap:
            self.printLine('\t^ $InterfaceWrapper forHandle: resultValue_', InterfaceWrapper=self.interfaceTypeMap[method.returnType])
        elif method.returnType == 'error':
            self.printLine('\tself checkErrorCode: resultValue_')
        else:
            self.printLine('\t^ resultValue_')

        self.endMethod()

    def emitBindings(self, api):
        self.emitBaseClasses(api)

    def beginMethod(self, className, category):
        self.printLine("!$ClassName methodsFor: '$Category'!", ClassName = className, Category = category)

    def endMethod(self):
        self.printLine("! !")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "make-headers <definitions> <output dir>"
    else:
        api = ApiDefinition.loadFromFileNamed(sys.argv[1])
        with open(sys.argv[2], 'w') as out:
            visitor = MakePharoBindingsVisitor(out)
            api.accept(visitor)
