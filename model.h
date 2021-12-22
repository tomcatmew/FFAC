#ifndef FFCC_MODEL_H
#define FFCC_MODEL_H

#include <torch/script.h>

template <int nIn>
class PFNN_net {
public:
    torch::jit::script::Module module;

public:
    PFNN_net(const std::string& path) {
        try {
            module = torch::jit::load(path);
        }
        catch (const c10::Error& e) {
            std::cerr << "error load model " << e.msg() << "\n";
            return;
        }
        module.eval();
    }

    template <typename T0>
    void predict(
        std::vector<T0>& last_vector,
        const std::vector<float>& aIn)
    {
        assert(aIn.size() == nIn);

        std::vector<float> aInFF;

        aInFF.assign(aIn.begin(), aIn.end());

        constexpr int nInFF = nIn;
        assert(aInFF.size() == nInFF);

        std::vector<torch::jit::IValue> inputs;
        inputs.emplace_back(
            torch::from_blob(const_cast<float*>(aInFF.data()), { 1,nInFF }));   // batch_size, input_size
        at::Tensor output_para = module.forward(inputs).toTensor();

        last_vector.clear();
        for (int i = 0; i < output_para.sizes()[1]; i++) {
            last_vector.push_back(output_para[0][i].item<T0>());
        }
    }
};



/**
* MLP_Fourier model
* @param aIn : Input of the model, phase and trajectory in local frame 
* @param nOctave : Fourier feature hyperparameter N, mutiply the positive interger up to N
*/
template <int nIn, int nOctave>
class MLP_Fourier{
public:
  torch::jit::script::Module module;

public:
  MLP_Fourier(const std::string& path) {
    try {
      module = torch::jit::load(path);
    }
    catch (const c10::Error &e) {
      std::cerr << "error load model " << e.msg() << "\n";
      return;
    }
    module.eval();
  }

  template <typename T0>
  void predict(
      std::vector<T0>& last_vector,
      const std::vector<float>& aIn)
  {
    assert(aIn.size() == nIn);

    std::vector<float> aInFF;
    if(nOctave==0){
      aInFF.assign(aIn.begin(),aIn.end());
    }
    else{
      float phase = aIn[0];
      for(unsigned int ioct=0;ioct<nOctave;++ioct){
        aInFF.push_back(sin(2*M_PI*phase*(1+ioct)));
        aInFF.push_back(cos(2*M_PI*phase*(1+ioct)));
      }
      aInFF.insert(aInFF.end(),aIn.begin()+1,aIn.end());
    }
    constexpr int nInFF = (nOctave==0) ? nIn : nIn-1+nOctave*2;
    assert(aInFF.size()==nInFF);

    std::vector<torch::jit::IValue> inputs;
    inputs.emplace_back(
        torch::from_blob(const_cast<float *>(aInFF.data()), {1,nInFF}));
    at::Tensor output_para = module.forward(inputs).toTensor();

    last_vector.clear();
    for (int i = 0; i < output_para.sizes()[1]; i++) {
      last_vector.push_back(output_para[0][i].item<T0>());
    }
  }
};


template <int nIn, int nLayer, int nHidden>
class RNN_Phase{
 public:
  torch::jit::script::Module module;
  std::vector<float> vector_hidden;

 public:
  RNN_Phase(const std::string& path) {
    try {
      module = torch::jit::load(path);
    }
    catch (const c10::Error &e) {
      std::cerr << "error load model " << e.msg() << "\n";
      return;
    }
    module.eval();
    vector_hidden.assign(nLayer*nHidden, 0.0);
  }

  void predict(
      std::vector<float>& vector_out,
      const std::vector<float>& aIn)
  {
    assert(aIn.size() == nIn);
    std::vector<torch::jit::IValue> inputs;
    inputs.emplace_back(
        torch::from_blob(
            const_cast<float *>(aIn.data()),
            {1,1,nIn}));  // seqlength, batch_size, input_size
    inputs.emplace_back(
        torch::from_blob(
            vector_hidden.data(),
            {nLayer,1,nHidden}));  // num_layer, batch_size, hidden_size
    torch::jit::IValue out_hidden = module.forward(inputs);
    c10::intrusive_ptr<c10::ivalue::Tuple> hoge = out_hidden.toTuple();
    at::Tensor out0 = hoge->elements()[0].toTensor();
    at::Tensor hid0 = hoge->elements()[1].toTensor();
    vector_out.assign(
        out0.template data_ptr<float>(),
            out0.template data_ptr<float>()+out0.numel() );
    vector_hidden.assign(
        hid0.template data_ptr<float>(),
            hid0.template data_ptr<float>()+hid0.numel() );

    /*
    std::cout << "hoge" << std::endl;
    for(auto f: vector_hidden ){
      std::cout << f << std::endl;
    }
     */
  }
};

#endif //INC_2_RUNTIME_MODEL_H
