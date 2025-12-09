import tensorrt as trt
import sys
import os

def build_engine(onnx_file_path, engine_file_path, fp16_mode=True, max_batch_size=1):
    """
    Build TensorRT engine from ONNX file.
    
    Args:
        onnx_file_path: Path to ONNX model
        engine_file_path: Output path for TensorRT engine
        fp16_mode: Enable FP16 precision (faster, slightly less accurate)
        max_batch_size: Maximum batch size
    """
    # Create TensorRT logger
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    
    # Create builder
    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, TRT_LOGGER)
    
    # Parse ONNX
    print(f"📖 Parsing ONNX file: {onnx_file_path}")
    with open(onnx_file_path, 'rb') as model:
        if not parser.parse(model.read()):
            print('❌ ERROR: Failed to parse ONNX file.')
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            return None
    
    print("✅ ONNX file parsed successfully")
    
    # Configure builder
    config = builder.create_builder_config()
    
    # Set memory pool limit (8 GB for RTX 4070)
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 8 * (1 << 30))
    
    # Enable FP16 mode for faster inference
    if fp16_mode and builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)
        print("✅ FP16 mode enabled")
    else:
        print("⚠️  FP16 mode not available, using FP32")
    
    # Build engine
    print("🔨 Building TensorRT engine... (this may take 2-5 minutes)")
    serialized_engine = builder.build_serialized_network(network, config)
    
    if serialized_engine is None:
        print("❌ ERROR: Failed to build TensorRT engine")
        return None
    
    # Save engine to file
    print(f"💾 Saving engine to: {engine_file_path}")
    with open(engine_file_path, 'wb') as f:
        f.write(serialized_engine)
    
    print("✅ TensorRT engine built successfully!")
    return engine_file_path

if __name__ == "__main__":
    onnx_path = "yolov8n-pose.onnx"
    engine_path = "yolov8n-pose.engine"
    
    # Build engine with FP16 precision
    build_engine(onnx_path, engine_path, fp16_mode=True)
    
    # Print file sizes
    onnx_size = os.path.getsize(onnx_path) / (1024 * 1024)
    engine_size = os.path.getsize(engine_path) / (1024 * 1024)
    
    print(f"\n📊 Model Sizes:")
    print(f"   ONNX:   {onnx_size:.2f} MB")
    print(f"   Engine: {engine_size:.2f} MB")
