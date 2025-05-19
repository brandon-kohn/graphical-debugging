using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphicalDebugging
{
    partial class ExpressionLoader
    {
        // Or ArrayPoint
        abstract class GXPoint : PointLoader
        {
            // memberArraySuffix has to start with '.'
            protected GXPoint(string memberArraySuffix, string coordType, Geometry.Traits traits)
            {
                this.memberArraySuffix = memberArraySuffix;
                this.coordType = coordType;
                if (coordType.StartsWith("boost::units::quantity"))
                    this.coordSuffix = ".val_";

                this.traits = traits;
                this.count = Math.Min(traits.Dimension, 2);
            }

            public override Geometry.Traits GetTraits(MemoryReader mreader, Debugger debugger,
                                                      string name)
            {
                return traits;
            }
            public string GetCoordType()
            {
                return coordType;
            }

            public string GetCoordSuffix()
            {
                return coordSuffix;
            }

            public override ExpressionDrawer.Point LoadPointParsed(Debugger debugger, string name, string type)
            {
                bool okx = true, oky = true;
                double x = 0, y = 0;
                string ptrName = name + memberArraySuffix;
                if (count >= 1)
                    okx = debugger.TryLoadDouble(ptrName + "[0]" + coordSuffix, out x);
                if (count >= 2)
                    oky = debugger.TryLoadDouble(ptrName + "[1]" + coordSuffix, out y);
                return Util.IsOk(okx, oky)
                     ? new ExpressionDrawer.Point(x, y)
                     : null;
            }

            public override ExpressionDrawer.Point LoadPointMemory(MemoryReader mreader, Debugger debugger,
                                                                      string name, string type)
            {
                string ptrName = name + memberArraySuffix;
                VariableInfo info = new VariableInfo(debugger, ptrName + "[0]" + coordSuffix);
                if (!info.IsValid)
                    return null;

                double[] values = new double[count];
                if (mreader.ReadNumericArray(info.Address, info.Type, info.Size, values))
                {
                    if (count >= 2)
                        return new ExpressionDrawer.Point(values[0], values[1]);
                    else if (count == 1)
                        return new ExpressionDrawer.Point(values[0], 0);
                    else
                        return null;
                }

                return null;
            }

            public override MemoryReader.Converter<double> GetMemoryConverter(MemoryReader mreader,
                                                                              Debugger debugger, // TODO - remove
                                                                              string name, string type)
            {
                // TODO: byteSize and byteOffset could be created in LoaderCreator
                string ptrName = name + memberArraySuffix;
                string elemName = ptrName + "[0]";
                if (!debugger.GetTypeSizeof(coordType, out int elemSize))
                    return null;
                MemoryReader.Converter<double> arrayConverter
                    = mreader.GetNumericArrayConverter(coordType, elemSize, count);
                return arrayConverter != null
                    && debugger.GetValueSizeof(name, out int byteSize)
                    && debugger.GetAddressOffset(name, elemName, out long byteOffset)
                    && !Debugger.IsInvalidOffset(byteSize, byteOffset)
                     ? new MemoryReader.StructConverter<double>(byteSize,
                            new MemoryReader.Member<double>(arrayConverter, (int)byteOffset))
                     : null;
            }

            private readonly string memberArraySuffix;
            private readonly string coordType;
            private readonly string coordSuffix;
            private readonly Geometry.Traits traits;
            private readonly int count;
        }

        class GeometrixPoint : GXPoint
        {
            public class LoaderCreator : ExpressionLoader.ILoaderCreator
            {
                public bool IsUserDefined() { return false; }
                public Kind Kind() { return ExpressionLoader.Kind.Point; }
                public Loader Create(Loaders loaders, Debugger debugger, string name, string type, string id)
                {
                    if (id != "geometrix::point")
                        return null;

                    List<string> tparams = Util.Tparams(type);
                    if (tparams.Count < 2)
                        return null;

                    string coordType = tparams[0];
                    int dimension = int.Parse(tparams[1]);

                    return new GeometrixPoint(coordType, new Geometry.Traits(dimension));
                }
            }

            protected GeometrixPoint(string coordType, Geometry.Traits traits)
                : base(".m_sequence.elems", coordType, traits)
            { }
        }
        class GeometrixVector : GXPoint
        {
            public class LoaderCreator : ExpressionLoader.ILoaderCreator
            {
                public bool IsUserDefined() { return false; }
                public Kind Kind() { return ExpressionLoader.Kind.Point; }
                public Loader Create(Loaders loaders, Debugger debugger, string name, string type, string id)
                {
                    if (id != "geometrix::vector")
                        return null;

                    List<string> tparams = Util.Tparams(type);
                    if (tparams.Count < 2)
                        return null;

                    string coordType = tparams[0];
                    int dimension = int.Parse(tparams[1]);

                    return new GeometrixVector(coordType, new Geometry.Traits(dimension));
                }
            }

            protected GeometrixVector(string coordType, Geometry.Traits traits)
                : base(".m_sequence.elems", coordType, traits)
            { }
        }

        class GeometrixBox : BoxLoader
        {
            public class LoaderCreator : ExpressionLoader.ILoaderCreator
            {
                public bool IsUserDefined() { return false; }
                public Kind Kind() { return ExpressionLoader.Kind.Box; }
                public Loader Create(Loaders loaders, Debugger debugger, string name, string type, string id)
                {
                    if (id != "geometrix::axis_aligned_bounding_box")
                        return null;

                    List<string> tparams = Util.Tparams(type);
                    if (tparams.Count < 1)
                        return null;

                    string m_min_corner = name + ".m_low";
                    string m_max_corner = name + ".m_high";

                    string pointType = tparams[0];
                    PointLoader pointLoader = loaders.FindByType(ExpressionLoader.Kind.Point,
                                                                 m_min_corner,
                                                                 pointType) as PointLoader;
                    return pointLoader != null
                        && debugger.GetTypeSizeof(type, out int sizeOf)
                        && debugger.GetAddressOffset(name, m_min_corner, out long minDiff)
                        && debugger.GetAddressOffset(name, m_max_corner, out long maxDiff)
                        && !Debugger.IsInvalidOffset(sizeOf, minDiff, maxDiff)
                         ? new GeometrixBox(pointLoader, pointType, sizeOf, minDiff, maxDiff)
                         : null;
                }
            }

            private GeometrixBox(PointLoader pointLoader, string pointType, int sizeOf, long minDiff, long maxDiff)
            {
                this.pointLoader = pointLoader;
                this.pointType = pointType;
                this.sizeOf = sizeOf;
                this.minDiff = minDiff;
                this.maxDiff = maxDiff;
            }

            public override Geometry.Traits GetTraits(MemoryReader mreader, Debugger debugger,
                                                      string name)
            {
                return pointLoader.GetTraits(mreader, debugger, name);
            }

            public override ExpressionDrawer.IDrawable Load(MemoryReader mreader, Debugger debugger,
                                                            string name, string type,
                                                            LoadCallback callback) // dummy callback
            {
                string m_min_corner = name + ".m_low";
                string m_max_corner = name + ".m_high";

                Geometry.Point fp = pointLoader.LoadPoint(mreader, debugger, m_min_corner, pointType);
                Geometry.Point sp = pointLoader.LoadPoint(mreader, debugger, m_max_corner, pointType);

                return Util.IsOk(fp, sp)
                     ? new ExpressionDrawer.Box(fp, sp)
                     : null;
            }

            public override MemoryReader.Converter<double> GetMemoryConverter(MemoryReader mreader,
                                                                              Debugger debugger, // TODO - remove
                                                                              string name, string type)
            {
                string m_min_corner = name + ".m_low";
                //string m_max_corner = name + ".m_max_corner";

                MemoryReader.Converter<double> pointConverter = pointLoader.GetMemoryConverter(mreader, debugger, m_min_corner, pointType);
                if (pointConverter == null)
                    return null;

                return new MemoryReader.StructConverter<double>(sizeOf,
                            new MemoryReader.Member<double>(pointConverter, (int)minDiff),
                            new MemoryReader.Member<double>(pointConverter, (int)maxDiff));
            }

            private readonly PointLoader pointLoader;
            private readonly string pointType;
            private readonly long minDiff;
            private readonly long maxDiff;
            private readonly int sizeOf;
        }

        class GeometrixSegment : SegmentLoader
        {
            public class LoaderCreator : ExpressionLoader.ILoaderCreator
            {
                public bool IsUserDefined() { return false; }
                public Kind Kind() { return ExpressionLoader.Kind.Segment; }
                public Loader Create(Loaders loaders, Debugger debugger, string name, string type, string id)
                {
                    if (id != "geometrix::segment")
                        return null;

                    List<string> tparams = Util.Tparams(type);
                    if (tparams.Count < 1)
                        return null;

                    string first = name + ".m_start";
                    string second = name + ".m_end";

                    string pointType = tparams[0];
                    PointLoader pointLoader = loaders.FindByType(ExpressionLoader.Kind.Point,
                                                                 first,
                                                                 pointType) as PointLoader;
                    return pointLoader != null
                        && debugger.GetTypeSizeof(type, out int sizeOf)
                        && debugger.GetAddressOffset(name, first, out long firstDiff)
                        && debugger.GetAddressOffset(name, second, out long secondDiff)
                        && !Debugger.IsInvalidOffset(sizeOf, firstDiff, secondDiff)
                         ? new GeometrixSegment(pointLoader, pointType, sizeOf, firstDiff, secondDiff)
                         : null;
                }
            }

            protected GeometrixSegment(PointLoader pointLoader, string pointType, int sizeOf, long firstDiff, long secondDiff)
            {
                this.pointLoader = pointLoader;
                this.pointType = pointType;
                this.sizeOf = sizeOf;
                this.firstDiff = firstDiff;
                this.secondDiff = secondDiff;
            }

            public override Geometry.Traits GetTraits(MemoryReader mreader, Debugger debugger,
                                                      string name)
            {
                return pointLoader.GetTraits(mreader, debugger, name);
            }

            public override ExpressionDrawer.IDrawable Load(MemoryReader mreader, Debugger debugger,
                                                            string name, string type,
                                                            LoadCallback callback) // dummy callback
            {
                string first = name + ".m_start";
                string second = name + ".m_end";

                Geometry.Point fp = pointLoader.LoadPoint(mreader, debugger, first, pointType);
                Geometry.Point sp = pointLoader.LoadPoint(mreader, debugger, second, pointType);

                return Util.IsOk(fp, sp)
                     ? new ExpressionDrawer.Segment(fp, sp)
                     : null;
            }

            public override MemoryReader.Converter<double> GetMemoryConverter(MemoryReader mreader,
                                                                              Debugger debugger, // TODO - remove
                                                                              string name, string type)
            {
                // NOTE: Because it can be created by derived class
                //   and these members can be set to invalid values
                //   e.g. GeometrixReferringSegment
                if (sizeOf <= 0
                 || Debugger.IsInvalidOffset(sizeOf, firstDiff, secondDiff))
                    return null;

                string first = name + ".m_start";
                MemoryReader.Converter<double> pointConverter = pointLoader.GetMemoryConverter(mreader, debugger, first, pointType);
                if (pointConverter == null)
                    return null;

                return new MemoryReader.StructConverter<double>(sizeOf,
                            new MemoryReader.Member<double>(pointConverter, (int)firstDiff),
                            new MemoryReader.Member<double>(pointConverter, (int)secondDiff));
            }

            private readonly PointLoader pointLoader;
            private readonly string pointType;
            private readonly long firstDiff;
            private readonly long secondDiff;
            private readonly int sizeOf;
        }

        class GeometrixNSphere : NSphereLoader
        {
            public class LoaderCreator : ExpressionLoader.ILoaderCreator
            {
                public bool IsUserDefined() { return false; }
                public Kind Kind() { return ExpressionLoader.Kind.NSphere; }
                public Loader Create(Loaders loaders, Debugger debugger, string name, string type, string id)
                {
                    if (id != "geometrix::sphere")
                        return null;

                    List<string> tparams = Util.Tparams(type);
                    if (tparams.Count < 2)
                        return null;

                    string m_center = name + ".m_center";
                    string m_radius = name + ".m_radius";

                    string pointType = tparams[1];
                    var pointLoader = loaders.FindByType(ExpressionLoader.Kind.Point,
                                                                 m_center,
                                                                 pointType) as GXPoint;

                    string radiusType = pointLoader.GetCoordType();

                    return pointLoader != null
                        && debugger.GetTypeSizeof(type, out int sizeOf)
                        && debugger.GetAddressOffset(name, m_center, out long centerDiff)
                        && debugger.GetAddressOffset(name, m_radius, out long radiusDiff)
                        && debugger.GetCppSizeof(radiusType, out int radiusSize)
                        && !Debugger.IsInvalidOffset(sizeOf, centerDiff, radiusDiff)
                         ? new GeometrixNSphere(pointLoader, pointType, radiusType, radiusSize, sizeOf, centerDiff, radiusDiff)
                         : null;
                }
            }

            private GeometrixNSphere(GXPoint pointLoader, string pointType, string radiusType, int radiusSize, int sizeOf, long centerDiff, long radiusDiff)
            {
                this.pointLoader = pointLoader;
                this.pointType = pointType;
                this.radiusType = radiusType;
                this.radiusSize = radiusSize;
                this.sizeOf = sizeOf;
                this.centerDiff = centerDiff;
                this.radiusDiff = radiusDiff;
            }

            public override Geometry.Traits GetTraits(MemoryReader mreader, Debugger debugger,
                                                      string name)
            {
                return pointLoader.GetTraits(mreader, debugger, name);
            }

            public override ExpressionDrawer.IDrawable Load(MemoryReader mreader, Debugger debugger,
                                                            string name, string type,
                                                            LoadCallback callback) // dummy callback
            {
                string m_center = name + ".m_center";
                string m_radius = name + ".m_radius" + pointLoader.GetCoordSuffix();

                Geometry.Point center = pointLoader.LoadPoint(mreader, debugger, m_center, pointType);
                bool ok = debugger.TryLoadDouble(m_radius, out double radius);

                return Util.IsOk(center, ok)
                     ? new ExpressionDrawer.NSphere(center, radius)
                     : null;
            }

            public override MemoryReader.Converter<double> GetMemoryConverter(MemoryReader mreader,
                                                                              Debugger debugger, // TODO - remove
                                                                              string name, string type)
            {
                // NOTE: In case it was created by derived class and these members set to invalid values
                if (sizeOf <= 0
                 || Debugger.IsInvalidOffset(sizeOf, centerDiff, radiusDiff))
                    return null;

                string m_center = name + ".m_center";
                MemoryReader.Converter<double> pointConverter = pointLoader.GetMemoryConverter(mreader, debugger, m_center, pointType);
                if (pointConverter == null)
                    return null;

                MemoryReader.Converter<double> radiusConverter = mreader.GetNumericConverter(radiusType, radiusSize);

                return new MemoryReader.StructConverter<double>(sizeOf,
                            new MemoryReader.Member<double>(pointConverter, (int)centerDiff),
                            new MemoryReader.Member<double>(radiusConverter, (int)radiusDiff));
            }

            private readonly GXPoint pointLoader;
            private readonly string pointType;
            private readonly string radiusType;
            private readonly int radiusSize;
            private readonly long centerDiff;
            private readonly long radiusDiff;
            private readonly int sizeOf;
        }
        static void GetGeometrixContainerInfo(string type,
                                       int pointTIndex,
                                       int containerTIndex,
                                       int allocatorTIndex,
                                       out string elementType,
                                       out string containerType)
        {
            elementType = "";
            containerType = "";

            List<string> tparams = Util.Tparams(type);
            if (tparams.Count < 2)
                return;

            elementType = tparams[0];
            var allocType = tparams[1];
            containerType = StdContainerType("std::vector",
                                             elementType,
                                             allocType);
        }
        class GeometrixRange<ResultType> : PointRange<ResultType>
            where ResultType : class
                             , ExpressionDrawer.IDrawable
                             , Geometry.IContainer<Geometry.Point>
                             , new()
        {
            public class LoaderCreator : ExpressionLoader.ILoaderCreator
            {
                public delegate Loader DerivedConstructor(ContainerLoader containerLoader, string containerType,
                                                          PointLoader pointLoader, string pointType);

                public LoaderCreator(Kind kind, string id,
                                     int pointTIndex, int containerTIndex, int allocatorTIndex,
                                     DerivedConstructor derivedConstructor)
                {
                    this.kind = kind;
                    this.id = id;
                    this.pointTIndex = pointTIndex;
                    this.containerTIndex = containerTIndex;
                    this.allocatorTIndex = allocatorTIndex;
                    this.derivedConstructor = derivedConstructor;
                }
                public bool IsUserDefined() { return false; }
                public Kind Kind() { return kind; }
                public Loader Create(Loaders loaders, Debugger debugger, string name, string type, string id)
                {
                    if (id != this.id)
                        return null;

                    GetGeometrixContainerInfo(type, pointTIndex, containerTIndex, allocatorTIndex,
                                       out string pointType, out string containerType);

                    ContainerLoader containerLoader = loaders.FindByType(ExpressionLoader.Kind.Container,
                                                                         name,
                                                                         containerType) as ContainerLoader;
                    if (containerLoader == null)
                        return null;

                    containerLoader.ElementInfo(name, containerType, out string pointName, out string _);

                    PointLoader pointLoader = loaders.FindByType(ExpressionLoader.Kind.Point,
                                                                 pointName,
                                                                 pointType) as PointLoader;
                    if (pointLoader == null)
                        return null;

                    return derivedConstructor(containerLoader, containerType, pointLoader, pointType);
                }

                private readonly Kind kind;
                private readonly string id;
                private readonly int pointTIndex;
                private readonly int containerTIndex;
                private readonly int allocatorTIndex;
                private readonly DerivedConstructor derivedConstructor;
            }

            protected GeometrixRange(ContainerLoader containerLoader, string containerType,
                              PointLoader pointLoader, string pointType)
            {
                this.containerLoader = containerLoader;
                this.containerType = containerType;
                this.pointLoader = pointLoader;
                this.pointType = pointType;
            }

            public override Geometry.Traits GetTraits(MemoryReader mreader, Debugger debugger,
                                                      string name)
            {
                return pointLoader.GetTraits(mreader, debugger, name);
            }

            public override ExpressionDrawer.IDrawable Load(MemoryReader mreader, Debugger debugger,
                                                            string name, string type,
                                                            LoadCallback callback)
            {
                ResultType result = null;

                if (mreader != null)
                {
                    containerLoader.ElementInfo(name, containerType, out string pointName, out string _);

                    result = LoadMemory(mreader, debugger, name, type,
                                        pointName, pointType, pointLoader,
                                        containerLoader, callback);
                }

                if (result == null)
                {
                    result = LoadParsed(mreader, debugger, name, type,
                                        pointType, pointLoader,
                                        containerLoader, callback);
                }

                return result;
            }

            readonly ContainerLoader containerLoader;
            readonly string containerType;
            readonly PointLoader pointLoader;
            readonly string pointType;
        }

        class GeometrixLinestring : GeometrixRange<ExpressionDrawer.Linestring>
        {
            public new class LoaderCreator : GeometrixRange<ExpressionDrawer.Linestring>.LoaderCreator
            {
                public LoaderCreator()
                    : base(ExpressionLoader.Kind.Linestring,
                           "geometrix::polyline",
                           0, 1, 2,
                           delegate (ContainerLoader containerLoader, string containerType,
                                     PointLoader pointLoader, string pointType)
                                     {
                                         return new GeometrixLinestring(containerLoader, containerType,
                                                                 pointLoader, pointType);
                                     })
                { }
            }

            private GeometrixLinestring(ContainerLoader containerLoader, string containerType,
                                 PointLoader pointLoader, string pointType)
                : base(containerLoader, containerType, pointLoader, pointType)
            { }
        }

        class GeometrixRing : GeometrixRange<ExpressionDrawer.Ring>
        {
            public new class LoaderCreator : GeometrixRange<ExpressionDrawer.Ring>.LoaderCreator
            {
                public LoaderCreator()
                    : base(ExpressionLoader.Kind.Ring,
                           "geometrix::polygon",
                           0, 3, 4,
                           delegate (ContainerLoader containerLoader, string containerType,
                                     PointLoader pointLoader, string pointType)
                                     {
                                         return new GeometrixRing(containerLoader, containerType,
                                                           pointLoader, pointType);
                                     })
                { }
            }

            private GeometrixRing(ContainerLoader containerLoader, string containerType,
                           PointLoader pointLoader, string pointType)
                : base(containerLoader, containerType, pointLoader, pointType)
            { }
        }

        class GeometrixPolygon : PolygonLoader
        {
            public class LoaderCreator : ExpressionLoader.ILoaderCreator
            {
                public bool IsUserDefined() { return false; }
                public Kind Kind() { return ExpressionLoader.Kind.Polygon; }
                public Loader Create(Loaders loaders, Debugger debugger, string name, string type, string id)
                {
                    if (id != "geometrix::polygon_with_holes")
                        return null;

                    string outerName = name + ".m_outer";
                    string innersName = name + ".m_holes";

                    string outerType = debugger.GetValueType(outerName);
                    if (outerType == null)
                        return null;
                    GeometrixRing outerLoader = loaders.FindByType(ExpressionLoader.Kind.Ring,
                                                            outerName, outerType) as GeometrixRing;
                    if (outerLoader == null)
                        return null;

                    string innersType = debugger.GetValueType(innersName);
                    if (innersType == null)
                        return null;
                    ContainerLoader innersLoader = loaders.FindByType(ExpressionLoader.Kind.Container,
                                                                      innersName, innersType) as ContainerLoader;
                    if (innersLoader == null)
                        return null;

                    return new GeometrixPolygon(outerLoader, outerType, innersLoader);
                }
            }

            // TODO: Should this be GeometrixRing or a generic ring Loader?
            private GeometrixPolygon(GeometrixRing outerLoader, string outerType,
                              ContainerLoader innersLoader)
            {
                this.outerLoader = outerLoader;
                this.outerType = outerType;
                this.innersLoader = innersLoader;
            }

            public override Geometry.Traits GetTraits(MemoryReader mreader, Debugger debugger,
                                                      string name)
            {
                return outerLoader.GetTraits(mreader, debugger, name);
            }

            public override ExpressionDrawer.IDrawable Load(MemoryReader mreader, Debugger debugger,
                                                            string name, string type,
                                                            LoadCallback callback)
            {
                string outerName = name + ".m_outer";
                string innersName = name + ".m_holes";

                ExpressionDrawer.Ring outer = outerLoader.Load(mreader, debugger,
                                                               outerName, outerType,
                                                               callback) as ExpressionDrawer.Ring;
                if (outer == null)
                    return null;

                List<Geometry.Ring> inners = new List<Geometry.Ring>();
                bool ok = innersLoader.ForEachElement(debugger, innersName, delegate (string elName)
                {
                    ExpressionDrawer.Ring inner = outerLoader.Load(mreader, debugger,
                                                                   elName, outerType,
                                                                   callback) as ExpressionDrawer.Ring;
                    if (inner == null)
                        return false;
                    inners.Add(inner);
                    //return callback();
                    return true;
                });

                return ok
                     ? new ExpressionDrawer.Polygon(outer, inners)
                     : null;
            }

            readonly GeometrixRing outerLoader;
            readonly string outerType;
            readonly ContainerLoader innersLoader;
        }
    }
}
