#pragma once

#include "IPlug_include_in_plug_hdr.h"

#include "NAM/dsp.h"
#include "AudioDSPTools/dsp/ResamplingContainer/ResamplingContainer.h"
#include "AudioDSPTools/dsp/wav.h"
#include "AudioDSPTools/dsp/ImpulseResponse.h"


const int kNumPresets = 1;
// The plugin is mono inside
constexpr size_t kNumChannelsInternal = 1;

enum EParams
{
    kInputLevel,
    kOutputLevel,
    kModelToggle,
    kIRToggle,
    kOutNormToggle,
    kNumParams
};

const int numKnobs = 2;

using namespace iplug;
using namespace igraphics;

// Get the sample rate of a NAM model.
// Sometimes, the model doesn't know its own sample rate; this wrapper guesses 48k based on the way that most
// people have used NAM in the past.
double GetNAMSampleRate(const std::unique_ptr<nam::DSP>& model)
{
    // Some models are from when we didn't have sample rate in the model.
    // For those, this wraps with the assumption that they're 48k models, which is probably true.
    const double assumedSampleRate = 48000.0;
    const double reportedEncapsulatedSampleRate = model->GetExpectedSampleRate();
    const double encapsulatedSampleRate =
        reportedEncapsulatedSampleRate <= 0.0 ? assumedSampleRate : reportedEncapsulatedSampleRate;
    return encapsulatedSampleRate;
};

class ResamplingNAM : public nam::DSP
{
public:
    // Resampling wrapper around the NAM models
    ResamplingNAM(std::unique_ptr<nam::DSP> encapsulated, const double expected_sample_rate)
        : nam::DSP(expected_sample_rate)
        , mEncapsulated(std::move(encapsulated))
        , mResampler(GetNAMSampleRate(mEncapsulated))
    {
        // Assign the encapsulated object's processing function  to this object's member so that the resampler can use it:
        auto ProcessBlockFunc = [&](NAM_SAMPLE** input, NAM_SAMPLE** output, int numFrames) {
            mEncapsulated->process(input[0], output[0], numFrames);
            };
        mBlockProcessFunc = ProcessBlockFunc;

        // Get the other information from the encapsulated NAM so that we can tell the outside world about what we're
        // holding.
        if (mEncapsulated->HasLoudness())
            SetLoudness(mEncapsulated->GetLoudness());

        // NOTE: prewarm samples doesn't mean anything--we can prewarm the encapsulated model as it likes and be good to
        // go.
        // _prewarm_samples = 0;

        // And be ready
        int maxBlockSize = 2048; // Conservative
        Reset(expected_sample_rate, maxBlockSize);
    };

    ~ResamplingNAM() = default;

    void prewarm() override { mEncapsulated->prewarm(); };

    void process(NAM_SAMPLE* input, NAM_SAMPLE* output, const int num_frames) override
    {
        if (num_frames > mMaxExternalBlockSize)
            // We can afford to be careful
            throw std::runtime_error("More frames were provided than the max expected!");

        if (!NeedToResample())
        {
            mEncapsulated->process(input, output, num_frames);
        }
        else
        {
            mResampler.ProcessBlock(&input, &output, num_frames, mBlockProcessFunc);
        }
    };

    int GetLatency() const { return NeedToResample() ? mResampler.GetLatency() : 0; };

    void Reset(const double sampleRate, const int maxBlockSize)
    {
        mExpectedSampleRate = sampleRate;
        mMaxExternalBlockSize = maxBlockSize;
        mResampler.Reset(sampleRate, maxBlockSize);

        // Allocations in the encapsulated model (HACK)
        // Stolen some code from the resampler; it'd be nice to have these exposed as methods? :)
        const double mUpRatio = sampleRate / GetEncapsulatedSampleRate();
        const auto maxEncapsulatedBlockSize = static_cast<int>(std::ceil(static_cast<double>(maxBlockSize) / mUpRatio));
        mEncapsulated->ResetAndPrewarm(sampleRate, maxEncapsulatedBlockSize);
    };

    // So that we can let the world know if we're resampling (useful for debugging)
    double GetEncapsulatedSampleRate() const { return GetNAMSampleRate(mEncapsulated); };

private:
    bool NeedToResample() const { return GetExpectedSampleRate() != GetEncapsulatedSampleRate(); };
    // The encapsulated NAM
    std::unique_ptr<nam::DSP> mEncapsulated;

    // The resampling wrapper
    dsp::ResamplingContainer<NAM_SAMPLE, 1, 12> mResampler;

    // Used to check that we don't get too large a block to process.
    int mMaxExternalBlockSize = 0;

    // This function is defined to conform to the interface expected by the iPlug2 resampler.
    std::function<void(NAM_SAMPLE**, NAM_SAMPLE**, int)> mBlockProcessFunc;
};

class NAMPluginSimplified final : public Plugin
{
public:
    NAMPluginSimplified(const InstanceInfo& info);
    ~NAMPluginSimplified();

    //#if IPLUG_DSP // http://bit.ly/2S64BDd
    void ProcessBlock(sample** inputs, sample** outputs, int nFrames) override;
    //#endif

private:
    // Sizes based on mInputArray
    size_t _GetBufferNumChannels() const;
    size_t _GetBufferNumFrames() const;

    // Allocates mInputPointers and mOutputPointers
    void _AllocateIOPointers(const size_t nChans);

    // Deallocates mInputPointers and mOutputPointers
    void _DeallocateIOPointers();

    // Manage pointers
    void _PrepareIOPointers(const size_t nChans);

    // Prepare the input & output buffers
    void _PrepareBuffers(const size_t numChannels, const size_t numFrames);

    // Moves DSP modules from staging area to the main area.
    // Also deletes DSP modules that are flagged for removal.
    // Exists so that we don't try to use a DSP module that's only
    // partially-instantiated.
    void _ApplyDSPStaging();

    // Model Staging
    std::string _StageModel(const WDL_String& modelPath);
    std::string _StageModelStream();

    // IR Staging
    dsp::wav::LoadReturnCode _StageIR(const WDL_String& irPath);
    dsp::wav::LoadReturnCode _StageIRStream();

    // Fallback that just copies inputs to outputs if mDSP doesn't hold a model.
    void _FallbackDSP(iplug::sample** inputs, iplug::sample** outputs, const size_t numChannels, const size_t numFrames);

    // Apply the normalization for the model output (if possible)
    void _NormalizeModelOutput(iplug::sample** buffer, const size_t numChannels, const size_t numFrames);

    // Copy the input buffer to the object, applying input level.
    // :param nChansIn: In from external
    // :param nChansOut: Out to the internal of the DSP routine
    void _ProcessInput(iplug::sample** inputs, const size_t nFrames, const size_t nChansIn, const size_t nChansOut);

    // Copy the output to the output buffer, applying output level.
    // :param nChansIn: In from internal
    // :param nChansOut: Out to external
    void _ProcessOutput(iplug::sample** inputs, iplug::sample** outputs, const size_t nFrames, const size_t nChansIn,
        const size_t nChansOut);

    // Member data

    // Input arrays to NAM
    std::vector<std::vector<iplug::sample>> mInputArray;
    // Output from NAM
    std::vector<std::vector<iplug::sample>> mOutputArray;
    // Pointer versions
    iplug::sample** mInputPointers = nullptr;
    iplug::sample** mOutputPointers = nullptr;

    // The current model
    std::unique_ptr<ResamplingNAM> mModel;

    // The staged model
    std::unique_ptr<ResamplingNAM> mStagedModel;

    // The current IR
    std::unique_ptr<dsp::ImpulseResponse> mIR;
    // The staged IR
    std::unique_ptr<dsp::ImpulseResponse> mStagedIR;

    // Path to model's config.json or model.nam
    WDL_String mNAMPath;
    // Path to IR (.wav file)
    WDL_String mIRPath;

    // Post-IR filters
    // recursive_linear_filter::HighPass mHighPass;
    //  recursive_linear_filter::LowPass mLowPass;
};